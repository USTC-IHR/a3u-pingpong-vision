"""Local browser test instrument: camera preview, PnP, FK, and scene endpoints."""
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import copy
import glob
import hashlib
import json
import mimetypes
import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np
import yaml

from .geometry import (ROOT, CHAIN, DEFAULT_URDF, RobotModel, load_config, marker_points,
                       metrics, project, solve_pose, table_corners, table_edges)
from .sources import DemoSource, OpenCVSource, ROSSource, detect_marker, marker_image


def demo_config():
    cfg = yaml.safe_load((ROOT/'configs/camera_a3u_face.yaml').read_text())
    cfg.update(camera_id='synthetic_face_left', image_size=[1280, 960], calibrated=True,
        camera_matrix=[[700, 0, 640], [0, 700, 480], [0, 0, 1]],
        distortion_coefficients=[0, 0, 0, 0, 0], calibration_source='SYNTHETIC_ONLY')
    cfg['marker']['enabled'] = True
    return cfg


class Experiment:
    def __init__(self, cfg, model, source, args):
        self.cfg, self.model, self.source, self.args = cfg, model, source, args
        self.lock = threading.RLock()
        self.cache = OrderedDict()
        self.pinned = None
        self.latest = None
        self.count = 0
        self.error = None
        self.stop = threading.Event()
        self.fixed = None
        if args.assume_zero_joints:
            self.fixed = {name: 0. for name in CHAIN}
        if args.joints:
            self.fixed = json.loads(Path(args.joints).read_text())
            if set(self.fixed) != set(CHAIN):
                raise ValueError('--joints JSON must contain exactly the five FK joint names (rad)')
        if self.fixed is not None:
            model.fk(self.fixed)
        self.output = Path(args.output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.log_lock = threading.Lock()
        self.log = (self.output/'observations.jsonl').open('a', buffering=1)
        # Freeze all configuration provenance; source data and runtime metadata are local-only.
        (self.output/'run-config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
        (self.output/'run.json').write_text(json.dumps(dict(source=args.source,
            camera_id=cfg['camera_id'], image_topic=args.image_topic, joint_topic=args.joint_topic,
            device=args.device, urdf=str(model.path), urdf_sha256=hashlib.sha256(model.path.read_bytes()).hexdigest(),
            fixed_joints=self.fixed, preview=args.preview, start_unix_ns=time.time_ns()), indent=2))

    def compute(self, item, objects, pixels, method):
        cfg = self.cfg
        t_optical_table, quality = solve_pose(objects, pixels, cfg)
        t_table_optical = np.linalg.inv(t_optical_table)
        warnings = []
        if not cfg.get('optical_extrinsic_verified'):
            warnings.append('CAD/optical axis alignment unverified')
        direct = cfg.get('optical_parent_link') is not None
        if direct and not cfg.get('factory_reference_mapping_verified'):
            warnings.append('Factory reference to URDF mapping / transform direction unverified')
        if cfg.get('intrinsics_field_verified') is False:
            warnings.append('Factory fisheye intrinsics not yet validated on held-out physical images')
        if not direct and cfg.get('lens_translation_mount_m') is None:
            warnings.append('Individual lens offset unknown; using CAD mount origin approximation')
        if quality['ambiguous']:
            warnings.append('Planar PnP ambiguous: best candidate is tentative')
        if item['frame'].positions is None:
            warnings.append('Robot pose unavailable: ' + item['frame'].joint_status)
        elif item['frame'].joint_status.startswith('assumed'):
            warnings.append('Fixed joint angles ASSUMED; keep all five chain joints stationary')
        result = dict(method=method, frame_id=item['id'], camera_id=cfg['camera_id'],
            capture_timestamp_ns=item['frame'].capture_ns,
            capture_clock=item['frame'].clock, received_monotonic_ns=item['frame'].received_ns,
            solved_monotonic_ns=time.monotonic_ns(), frame='table_surface_center_x_far_y_left_z_up',
            quality=quality, warnings=warnings, joint_status=item['frame'].joint_status,
            image_points_px=np.asarray(pixels).tolist(), object_points_table_m=np.asarray(objects).tolist(),
            T_optical_table=t_optical_table.tolist(), T_table_optical=t_table_optical.tolist(),
            camera_to_table_center_3d_m=float(np.linalg.norm(t_optical_table[:3, 3])),
            table_outline_px=project(table_corners(cfg), t_optical_table, cfg).tolist(),
            table_edges_px=project(table_edges(cfg), t_optical_table, cfg).tolist(),
            T_table_pelvis=None, link_matrices=None, distances=None,
            valid=not quality['ambiguous'], synthetic=self.args.source == 'demo',
            camera_pose_valid=not quality['ambiguous'], robot_pose_valid=False, robot_pose_grade='unavailable',
            image_sha256=hashlib.sha256(item['jpeg']).hexdigest())
        q = item['frame'].positions
        if q is not None:
            try:
                t_pelvis_optical = self.model.optical_pose(q, cfg)
                t_table_pelvis = t_table_optical @ np.linalg.inv(t_pelvis_optical)
                result.update(T_table_pelvis=t_table_pelvis.tolist(), distances=metrics(t_table_pelvis, cfg),
                    link_matrices={k: (t_table_pelvis @ v).tolist() for k, v in self.model.fk(q).items()})
                calibrated = bool(cfg.get('optical_extrinsic_verified') and (
                    cfg.get('factory_reference_mapping_verified') if direct else
                    cfg.get('lens_translation_mount_m') is not None))
                calibrated = calibrated and cfg.get('intrinsics_field_verified', True)
                assumed = item['frame'].joint_status.startswith('assumed')
                result['robot_pose_valid'] = bool(calibrated and not assumed and not quality['ambiguous'] and not result['synthetic'])
                result['robot_pose_grade'] = 'synthetic' if result['synthetic'] else ('assumed_fixed' if assumed else ('calibrated' if calibrated else ('factory_unverified' if direct else 'nominal_CAD')))
            except ValueError as exc:
                result['warnings'].append(str(exc))
        return result

    def record(self, result):
        # Full meshes are display data, not repeated in each observation.
        row = {k: v for k, v in result.items() if k != 'link_matrices'}
        with self.log_lock:
            self.log.write(json.dumps(row, allow_nan=False)+'\n')

    def capture_loop(self):
        while not self.stop.is_set():
            try:
                frame = self.source.read()
                if frame is None:
                    continue
                h, w = frame.image.shape[:2]
                if [w, h] != self.cfg['image_size']:
                    raise ValueError(f'Image is {w}x{h}; calibrated/configured size is {self.cfg["image_size"]}. No silent resizing allowed')
                if self.fixed is not None:
                    frame.positions = dict(self.fixed)
                    frame.joint_status = 'assumed_fixed_explicit'
                ok, jpeg = cv2.imencode('.jpg', frame.image, [cv2.IMWRITE_JPEG_QUALITY, 92])
                if not ok:
                    raise ValueError('JPEG encoding failed')
                self.count += 1
                item = dict(id=self.count, frame=frame, jpeg=jpeg.tobytes(), result=None, detection='not_initialized')
                if self.cfg['marker']['enabled'] and not self.args.preview:
                    try:
                        objects, pixels = detect_marker(frame.image, self.cfg)
                        item['result'] = self.compute(item, objects, pixels, 'aruco')
                        self.record(item['result'])
                        item['detection'] = 'detected'
                    except ValueError as exc:
                        item['detection'] = str(exc)
                        self.record(dict(valid=False, camera_pose_valid=False, robot_pose_valid=False,
                            frame_id=item['id'], camera_id=self.cfg['camera_id'],
                            capture_timestamp_ns=frame.capture_ns, capture_clock=frame.clock,
                            received_monotonic_ns=frame.received_ns, reason=str(exc)))
                # Keep compressed frames for annotation, not hundreds of raw 4 MB arrays.
                frame.image = None
                with self.lock:
                    self.cache[item['id']] = item
                    while len(self.cache) > 300:
                        self.cache.popitem(last=False)
                    self.latest = item
                    self.error = None
                if self.args.record_images:
                    (self.output/f'frame-{item["id"]:08d}.jpg').write_bytes(item['jpeg'])
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                if self.stop.wait(.2):
                    break

    def snapshot(self):
        with self.lock:
            item = self.latest
            age = (time.monotonic_ns()-item['frame'].received_ns)/1e9 if item else None
            stale = age is not None and age > self.args.stale_s and self.args.source != 'image'
            result = copy.deepcopy(item['result']) if item else None
            if result and (stale or self.error):
                result['valid'] = False
                result['camera_pose_valid'] = result['robot_pose_valid'] = False
            return dict(frame_id=item['id'] if item else None, age_s=age,
                status='error' if self.error else ('waiting' if item is None else ('stale' if stale else ('offline' if self.args.source == 'image' else 'live'))),
                error=self.error, detection=item['detection'] if item else None, result=result,
                source=self.args.source, camera_id=self.cfg['camera_id'], preview=self.args.preview,
                table=self.cfg['table'], image_size=self.cfg['image_size'])

    def get_frame(self, frame_id):
        with self.lock:
            if self.pinned and self.pinned['id'] == frame_id:
                return self.pinned
            if frame_id not in self.cache:
                raise ValueError('Frame expired; resume preview and freeze a fresh frame')
            return self.cache[frame_id]

    def save(self, frame_id):
        item = self.get_frame(frame_id)
        path = self.output/f'frame-{frame_id:08d}.jpg'
        path.write_bytes(item['jpeg'])
        return dict(saved=str(path))

    def manual(self, body):
        if self.args.preview:
            raise ValueError('Preview only: calibrate first')
        item = self.get_frame(int(body['frame_id']))
        result = self.compute(item, table_corners(self.cfg), body['points'], 'manual_snapshot')
        result['warnings'].append('Frozen single-frame annotation; NOT a live tracking result')
        self.save(item['id'])
        self.record(result)
        return result


def handler_for(experiment):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, data, mime='application/json', status=200):
            if mime == 'application/json':
                data = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            try:
                path = urlparse(self.path).path
                if path == '/api/state':
                    return self.send(experiment.snapshot())
                if path == '/api/model':
                    return self.send(dict(links=experiment.model.links))
                if path.startswith('/api/frame/'):
                    return self.send(experiment.get_frame(int(path.rsplit('/', 1)[1]))['jpeg'], 'image/jpeg')
                if path.startswith('/mesh/'):
                    file = experiment.model.meshes.get(path.rsplit('/', 1)[1])
                else:
                    file = (ROOT/'web'/('index.html' if path == '/' else path.lstrip('/'))).resolve()
                    if ROOT/'web' not in file.parents:
                        file = None
                if file is None or not file.is_file():
                    return self.send(dict(error='not found'), status=404)
                self.send(file.read_bytes(), mimetypes.guess_type(str(file))[0] or 'application/octet-stream')
            except (ValueError, KeyError) as exc:
                self.send(dict(error=str(exc)), status=400)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            try:
                origin = self.headers.get('Origin')
                if origin and urlparse(origin).netloc != self.headers.get('Host'):
                    return self.send(dict(error='same-origin requests required'), status=403)
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 16384:
                    raise ValueError('Invalid body size')
                body = json.loads(self.rfile.read(size))
                if self.path == '/api/solve':
                    return self.send(experiment.manual(body))
                if self.path == '/api/save':
                    return self.send(experiment.save(int(body['frame_id'])))
                if self.path == '/api/pin':
                    item = experiment.get_frame(int(body['frame_id']))
                    with experiment.lock:
                        experiment.pinned = item
                    return self.send(dict(frame_id=item['id'], pinned=True))
                self.send(dict(error='not found'), status=404)
            except (ValueError, KeyError, TypeError) as exc:
                self.send(dict(error=str(exc)), status=400)
    return Handler


def calibrate(args):
    files = sorted(glob.glob(args.images))
    pattern = (args.columns, args.rows)
    if min(pattern) < 3 or args.square_m <= 0:
        raise ValueError('At least 3x3 inner corners and positive square size required')
    objects = np.zeros((pattern[0]*pattern[1], 3), np.float32)
    objects[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)*args.square_m
    all_objects, all_pixels, used = [], [], []
    image_size = None
    for file in files:
        image = cv2.imread(file)
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        if image_size is not None and image_size != size:
            raise ValueError('Calibration images must all have the same resolution')
        image_size = size
        found, points = cv2.findChessboardCorners(gray, pattern)
        if not found:
            continue
        points = cv2.cornerSubPix(gray, points, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
        all_objects.append(objects.copy())
        all_pixels.append(points)
        used.append(file)
    if len(used) < 12:
        raise ValueError(f'Only {len(used)} usable views; need >=12 diverse checkerboard views (recommend 20-30)')
    distortion_model = getattr(args, 'distortion_model', 'plumb_bob')
    if distortion_model == 'fisheye':
        all_objects = [obj.astype(float).reshape(-1, 1, 3) for obj in all_objects]
        all_pixels = [pix.astype(float).reshape(-1, 1, 2) for pix in all_pixels]
        rms, k, d, rotations, translations = cv2.fisheye.calibrate(
            all_objects, all_pixels, image_size, None, None,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW)
    else:
        rms, k, d, rotations, translations = cv2.calibrateCamera(all_objects, all_pixels, image_size, None, None)
    errors = []
    for obj, pix, rv, tv in zip(all_objects, all_pixels, rotations, translations):
        projector = cv2.fisheye.projectPoints if distortion_model == 'fisheye' else cv2.projectPoints
        projected = projector(obj, rv, tv, k, d)[0]
        errors.append(float(np.sqrt(np.mean(np.sum((projected-pix)**2, axis=2)))))
    if not np.isfinite(k).all() or not np.isfinite(d).all() or rms > args.max_rms_px:
        raise ValueError(f'Calibration RMS {rms:.3f}px exceeds {args.max_rms_px}px or parameters invalid')
    cfg = yaml.safe_load(Path(args.template).read_text())
    if cfg.get('camera_id') != args.camera_id:
        for key in ('optical_parent_link', 'T_parent_optical', 'urdf_sha256',
                    'factory_reference_frame', 'factory_reference_mapping_verified',
                    'factory_transform_convention', 'factory_calib_time', 'factory_inputs'):
            cfg.pop(key, None)
        cfg.update(optical_extrinsic_verified=False, lens_translation_mount_m=None,
                   delta_mount_xyz_m=[0, 0, 0], delta_mount_rpy_rad=[0, 0, 0])
    cfg.update(camera_id=args.camera_id, image_size=list(image_size), camera_matrix=k.tolist(),
        distortion_coefficients=d.reshape(-1).tolist(), distortion_model=distortion_model, calibrated=True,
        intrinsics_field_verified=False,
        calibration_source='offline_checkerboard', calibration_rms_px=float(rms),
        calibration_views=len(used), calibration_square_m=args.square_m,
        calibration_per_view_rms_px=errors)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(cfg, sort_keys=False))
    output.with_suffix('.views.json').write_text(json.dumps(used, indent=2))
    print(f'{len(used)} views, RMS={rms:.3f}px; saved {output}. Validate on held-out views; low RMS alone does not prove calibration accuracy.')


def main():
    parser = argparse.ArgumentParser(description='A3U single face camera / table localization (read-only)')
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run', help='Serve preview / localization / robot-table 3D scene')
    run.add_argument('--source', choices=['demo', 'image', 'v4l2', 'ros'], required=True)
    run.add_argument('--config', default=str(ROOT/'configs/camera_a3u_face.yaml'))
    run.add_argument('--urdf', default=str(DEFAULT_URDF))
    run.add_argument('--device')
    run.add_argument('--image')
    run.add_argument('--image-topic')
    run.add_argument('--compressed', action='store_true')
    run.add_argument('--joint-topic')
    run.add_argument('--ros-clock-confirmed', action='store_true')
    fixed = run.add_mutually_exclusive_group()
    fixed.add_argument('--assume-zero-joints', action='store_true')
    fixed.add_argument('--joints', help='JSON object containing five static FK joint angles in rad')
    run.add_argument('--preview', action='store_true', help='Uncalibrated preview and calibration image saving only')
    run.add_argument('--size', nargs=2, type=int, metavar=('WIDTH', 'HEIGHT'), help='Required in --preview mode')
    run.add_argument('--record-images', action='store_true')
    run.add_argument('--host', default='127.0.0.1')
    run.add_argument('--port', type=int, default=8092)
    run.add_argument('--stale-s', type=float, default=.7)
    run.add_argument('--output', default=str(ROOT/'logs'/time.strftime('face-table-%Y%m%d-%H%M%S')))
    cal = sub.add_parser('calibrate', help='Offline checkerboard intrinsics, independent of mocap')
    cal.add_argument('--images', required=True, help='Quoted glob of saved calibration images')
    cal.add_argument('--columns', type=int, default=9, help='INNER corners, not squares')
    cal.add_argument('--rows', type=int, default=6)
    cal.add_argument('--square-m', type=float, required=True)
    cal.add_argument('--camera-id', required=True)
    cal.add_argument('--template', default=str(ROOT/'configs/camera_a3u_face.yaml'))
    cal.add_argument('--output', required=True)
    cal.add_argument('--max-rms-px', type=float, default=1.5)
    cal.add_argument('--distortion-model', choices=['plumb_bob', 'fisheye'], default='plumb_bob')
    factory = sub.add_parser('import-factory', help='Import ADU front-left factory candidate, not field-verified')
    factory.add_argument('--sensor', default=str(ROOT/'robot_param/sensor_param.yaml'))
    factory.add_argument('--hal', default=str(ROOT/'robot_param/hal/rgb_camera_params.yaml'))
    factory.add_argument('--urdf', default=str(ROOT/'assets/robot_model/a3_ultra_t2d5/urdf/model.urdf'))
    factory.add_argument('--output', default=str(ROOT/'configs/camera_a3u_front_left_fisheye.yaml'))
    marker = sub.add_parser('marker', help='Export the configured ArUco square with a white margin')
    marker.add_argument('--config', default=str(ROOT/'configs/camera_a3u_face.yaml'))
    marker.add_argument('--output', default=str(ROOT/'data/calib/table-marker.png'))
    discover = sub.add_parser('discover', help='List local video devices and optionally ROS image/joint topics (read-only)')
    discover.add_argument('--ros', action='store_true')
    args = parser.parse_args()
    if args.command == 'import-factory':
        from .factory import import_factory
        return import_factory(args)
    if args.command == 'discover':
        devices = []
        for entry in sorted(Path('/sys/class/video4linux').glob('video*')):
            devices.append(dict(device='/dev/'+entry.name, name=(entry/'name').read_text().strip()))
        result = dict(video_devices=devices)
        if args.ros:
            import rclpy
            rclpy.init(args=[])
            node = rclpy.create_node('a3u_face_table_discovery_readonly')
            try:
                deadline = time.monotonic()+3
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.2)
                types = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
                         'sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/JointState')
                result['ros_topics'] = [dict(topic=n, types=t) for n,t in node.get_topic_names_and_types()
                                        if any(v in types for v in t)]
            finally:
                node.destroy_node()
                rclpy.shutdown()
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return
    if args.command == 'calibrate':
        return calibrate(args)
    if args.command == 'marker':
        cfg = yaml.safe_load(Path(args.config).read_text())
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        image = cv2.copyMakeBorder(marker_image(cfg), 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
        if not cv2.imwrite(str(output), image):
            raise ValueError('Cannot save marker')
        print(f'Saved {output}; scale BLACK square to {cfg["marker"]["size_m"]} m, excluding white margin.')
        return
    if args.stale_s <= 0:
        raise ValueError('--stale-s must be positive')
    if args.joint_topic and (args.joints or args.assume_zero_joints or args.source != 'ros'):
        raise ValueError('Joint topic is ROS-only and cannot be combined with assumed static joints')
    if args.source == 'image' and not args.image:
        raise ValueError('--image is required')
    if args.source == 'demo':
        cfg = demo_config()
    elif args.preview:
        if not args.size or min(args.size) <= 0:
            raise ValueError('--preview requires --size WIDTH HEIGHT')
        cfg = yaml.safe_load(Path(args.config).read_text())
        cfg['image_size'] = args.size
        cfg['calibrated'] = False
    else:
        cfg = load_config(args.config)
    model = RobotModel(args.urdf)
    model.validate_camera_config(cfg)
    source = DemoSource(model, cfg) if args.source == 'demo' else (
        ROSSource(args) if args.source == 'ros' else OpenCVSource(args.source, args.device, args.image, cfg))
    experiment = Experiment(cfg, model, source, args)
    server = ThreadingHTTPServer((args.host, args.port), handler_for(experiment))
    thread = threading.Thread(target=experiment.capture_loop, daemon=True)
    thread.start()
    print(f'Face table experiment: http://{args.host}:{server.server_port} | {args.source} | {cfg["camera_id"]}', flush=True)
    print(f'Local output: {experiment.output} | Ctrl+C to stop. No robot commands are published.', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        experiment.stop.set()
        source.close()
        thread.join(timeout=3)
        server.server_close()
        if not thread.is_alive():
            experiment.log.close()


if __name__ == '__main__':
    main()
