import copy
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import cv2
import numpy as np
import yaml

from face_table.app import Experiment, calibrate, demo_config, handler_for
from face_table.geometry import (CHAIN, RobotModel, interpolate_joints, load_config,
    marker_points, metrics, project, rigid, solve_pose, table_corners, transform)
from face_table.sources import DemoSource, OpenCVSource, Frame, decode_raw, detect_marker, normalize_joints


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotModel()
        cls.cfg = demo_config()
        cls.source = DemoSource(cls.model, cls.cfg)

    def test_spec_zero_fk_and_optical_axes(self):
        frames = self.model.fk({n: 0. for n in CHAIN})
        np.testing.assert_allclose(frames[self.model.camera_link][:3, 3], [.0941768, 0, .49213084], atol=2e-7)
        cam = self.model.optical_pose({n: 0. for n in CHAIN}, self.cfg)
        # Optical forward must point robot-forward AND down, not robot-up.
        self.assertGreater(cam[0, 2], .98)
        self.assertLess(cam[2, 2], -.17)
        rigid(cam)

    def test_roundtrip_fk_pnp_nonzero_joints_distortion(self):
        cfg = copy.deepcopy(self.cfg)
        cfg['distortion_coefficients'] = [.08, -.03, .002, -.003, .01]
        cfg['lens_translation_mount_m'] = [.003, .032, -.001]
        cfg['delta_mount_rpy_rad'] = [.01, -.02, .005]
        q = dict(zip(CHAIN, [.12, -.04, .05, -.13, .12]))
        truth = transform([-2.6, .11, .42], [.02, -.03, .08])
        optical = self.model.optical_pose(q, cfg)
        t = np.linalg.inv(truth @ optical)
        pixels = project(table_corners(cfg), t, cfg)
        resolved, quality = solve_pose(table_corners(cfg), pixels, cfg)
        np.testing.assert_allclose(np.linalg.inv(resolved) @ np.linalg.inv(optical), truth, atol=1e-7)
        self.assertLess(quality['reprojection_rms_px'], 1e-4)
        self.assertFalse(quality['ambiguous'])

    def test_bad_points_and_missing_joint_rejected(self):
        for points in ([[1,1],[2,2],[3,3],[4,4]], [[10,10],[500,500],[500,10],[10,500]],
                       [[float('nan'),10],[500,10],[500,500],[10,500]]):
            with self.assertRaises(ValueError):
                solve_pose(table_corners(self.cfg), points, self.cfg)
        for q in ({}, {n: float('nan') for n in CHAIN}, {n: 99 for n in CHAIN}):
            with self.assertRaises(ValueError):
                self.model.optical_pose(q, self.cfg)

    def test_distance_definitions(self):
        result = metrics(transform([-2.2, .12, .45]), self.cfg)
        self.assertAlmostEqual(result['pelvis_to_table_rectangle_xy_m'], .83)
        self.assertAlmostEqual(result['near_end_signed_gap_x_m'], .83)
        self.assertAlmostEqual(metrics(transform([0,0,1]), self.cfg)['pelvis_to_table_rectangle_xy_m'], 0)
        result = metrics(transform([-2.37, 1.7625, .4]), self.cfg)
        self.assertAlmostEqual(result['pelvis_to_table_rectangle_xy_m'], np.sqrt(2))

    def test_joint_capture_brackets_no_hold(self):
        a, b = {n:0. for n in CHAIN}, {n:.2 for n in CHAIN}
        mid = interpolate_joints([(100, a), (200, b)], 150)
        self.assertAlmostEqual(mid['head_yaw_joint'], .1)
        for stamp in (90, 201):
            with self.assertRaises(ValueError):
                interpolate_joints([(100,a),(200,b)], stamp)
        with self.assertRaises(ValueError):
            interpolate_joints([(100,a),(200_000_000,b)], 150)
        with self.assertRaises(ValueError):
            interpolate_joints([(100,{}),(200,b)], 150)

    def test_unknown_intrinsics_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'cfg.yaml'
            for key, value in [('calibrated', False), ('distortion_model', 'equidistant'),
                               ('T_mount_optical', np.zeros((4,4)).tolist())]:
                cfg = copy.deepcopy(self.cfg)
                cfg['calibration_source'] = 'offline_checkerboard'
                cfg[key] = value
                path.write_text(yaml.safe_dump(cfg))
                with self.assertRaises(ValueError):
                    load_config(path)
            path.write_text(yaml.safe_dump(self.cfg))
            with self.assertRaises(ValueError):
                load_config(path)

    def test_synthetic_marker_full_pipeline(self):
        frame = self.source.read()
        objects, pixels = detect_marker(frame.image, self.cfg)
        t, quality = solve_pose(objects, pixels, self.cfg)
        root = np.linalg.inv(t) @ np.linalg.inv(self.model.optical_pose(frame.positions, self.cfg))
        np.testing.assert_allclose(root[:3,3], self.source.truth[:3,3], atol=.02)
        self.assertLess(quality['reprojection_rms_px'], 1)

    def test_raw_ros_stride_rgb(self):
        msg = SimpleNamespace(encoding='rgb8', width=2, height=1, step=8,
                              data=bytes([255,0,0,0,255,0,99,99]))
        image = decode_raw(msg)
        np.testing.assert_equal(image, [[[0,0,255],[0,255,0]]])
        msg.encoding='16UC1'
        with self.assertRaises(ValueError):
            decode_raw(msg)

    def test_alias_duplicate_and_missing_values(self):
        q = normalize_joints(['ultra_head_pitch_joint'], [.1])
        self.assertEqual(q, {'head_pitch_joint':.1})
        for names, values in [(['head_pitch_joint','ultra_head_pitch_joint'],[.1,.2]),
                              (['head_pitch_joint'],[])]:
            q = normalize_joints(names, values)
            self.assertTrue(np.isnan(q['head_pitch_joint']))

    def test_offline_source_has_no_fake_exposure_stamp(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'still.png'
            cv2.imwrite(str(path),np.zeros((40,60,3),np.uint8))
            source = OpenCVSource('image',image=str(path))
            frame = source.read()
            self.assertIsNone(frame.capture_ns)
            self.assertEqual(frame.image.shape,(40,60,3))
            self.assertIn('no_capture_timestamp',frame.clock)
            self.assertIsNone(source.read())
            source.close()


class CalibrationTests(unittest.TestCase):
    def test_rendered_checkerboard_calibration(self):
        self.render_and_calibrate('plumb_bob')

    def test_rendered_fisheye_checkerboard_calibration(self):
        self.render_and_calibrate('fisheye')

    def render_and_calibrate(self, distortion_model):
        # Render 15 distinct projective views with independent ground-truth K.
        # Exercises real corner finding + calibration, not a mocked cv2 result.
        from face_table.geometry import ROOT
        cfg = demo_config()
        board = np.zeros((420,600),np.uint8)
        for y in range(7):
            for x in range(10):
                board[y*60:(y+1)*60,x*60:(x+1)*60] = 255 if (x+y)%2 else 0
        boundary = np.array([[-.025,-.025,0],[.225,-.025,0],[.225,.15,0],[-.025,.15,0]])
        source = np.array([[0,0],[599,0],[599,419],[0,419]],np.float32)
        if distortion_model == 'fisheye':
            # Render a distorted image by inverse ray mapping of the perspective
            # checkerboard, rather than faking fisheye with a homography.
            yy, xx = np.mgrid[:960, :1280]
            pixels = np.stack((xx, yy), axis=-1).astype(float).reshape(-1, 1, 2)
            k = np.asarray(cfg['camera_matrix'], float)
            rays = cv2.fisheye.undistortPoints(pixels, k, np.array([-.05, -.007, -.004, .0007]))
            maps = (rays.reshape(960, 1280, 2)*[k[0,0], k[1,1]] + k[:2,2]).astype(np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            for i in range(15):
                t = transform([-.1+(i%3)*.04,-.07+(i%4)*.025,.55+(i%5)*.06],
                              [(-1)**i*(.12+.04*(i%3)), .15-.05*(i%4), -.1+.035*i])
                target = project(boundary,t,cfg).astype(np.float32)
                h = cv2.getPerspectiveTransform(source,target)
                image = cv2.warpPerspective(board,h,(1280,960),borderValue=180)
                if distortion_model == 'fisheye':
                    image = cv2.remap(image, maps[:,:,0], maps[:,:,1], cv2.INTER_LINEAR, borderValue=180)
                cv2.imwrite(str(Path(tmp)/f'view-{i:02d}.png'),image)
            output = Path(tmp)/'camera.yaml'
            calibrate(SimpleNamespace(images=tmp+'/*.png',columns=9,rows=6,square_m=.025,
                camera_id='test_right',template=str(ROOT/'configs/camera_a3u_face.yaml'),
                output=str(output),max_rms_px=1.5, distortion_model=distortion_model))
            calibrated = load_config(output)
            self.assertEqual(calibrated['calibration_views'],15)
            self.assertAlmostEqual(calibrated['camera_matrix'][0][0],700,delta=35)
            self.assertAlmostEqual(calibrated['camera_matrix'][1][1],700,delta=35)
            self.assertIsNone(calibrated['lens_translation_mount_m'])
            self.assertEqual(calibrated['distortion_model'], distortion_model)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg = demo_config()
        self.model = RobotModel()
        self.source = DemoSource(self.model, self.cfg)
        args = SimpleNamespace(assume_zero_joints=False, joints=None, source='demo',
            output=self.tmp.name, image_topic=None, joint_topic=None, device=None,
            preview=False, record_images=False, stale_s=.7)
        self.exp = Experiment(self.cfg, self.model, self.source, args)
        frame = self.source.read()
        self.exp.cache[1] = dict(id=1, frame=frame,
            jpeg=cv2.imencode('.jpg', frame.image)[1].tobytes(), result=None, detection='test')
        self.exp.latest = self.exp.cache[1]
        self.server = ThreadingHTTPServer(('127.0.0.1',0),handler_for(self.exp))
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown()
        self.thread.join()
        self.server.server_close()
        self.exp.log.close()
        self.tmp.cleanup()

    def get(self,path):
        return urlopen(self.url+path)

    def post(self,path,body):
        return urlopen(Request(self.url+path,data=json.dumps(body).encode(),headers={'Content-Type':'application/json'}))

    def test_snapshot_solve_artifacts_and_expired_id(self):
        state = json.load(self.get('/api/state'))
        self.assertEqual(state['frame_id'], 1)
        pixels = project(table_corners(self.cfg), self.source.t_camera_table, self.cfg)
        result = json.load(self.post('/api/solve',dict(frame_id=1,points=pixels.tolist())))
        self.assertEqual(result['method'], 'manual_snapshot')
        np.testing.assert_allclose(result['distances']['pelvis_table_xyz_m'],[-2.2,.12,.45], atol=1e-7)
        self.assertTrue((Path(self.tmp.name)/'frame-00000001.jpg').is_file())
        self.assertIn('manual_snapshot',(Path(self.tmp.name)/'observations.jsonl').read_text())
        self.post('/api/pin',dict(frame_id=1)).read()
        self.exp.cache.clear()
        self.assertGreater(len(self.get('/api/frame/1').read()),100)
        with self.assertRaises(HTTPError) as exc:
            self.post('/api/solve',dict(frame_id=999,points=pixels.tolist()))
        self.assertEqual(exc.exception.code,400)

    def test_camera_only_and_stale_invalidation(self):
        item = self.exp.latest
        item['frame'].positions = None
        result = self.exp.compute(item,table_corners(self.cfg),
            project(table_corners(self.cfg), self.source.t_camera_table,self.cfg),'test')
        self.assertIsNone(result['T_table_pelvis'])
        self.assertIsNone(result['distances'])
        item['result'] = result
        item['frame'].received_ns = time.monotonic_ns()-2_000_000_000
        state = self.exp.snapshot()
        self.assertEqual(state['status'],'stale')
        self.assertFalse(state['result']['valid'])

    def test_marker_loss_does_not_hold_previous_pose(self):
        first = self.exp.latest['frame']
        second = Frame(np.full_like(first.image,127))
        exp = self.exp
        class SequenceSource:
            count = 0
            def read(self):
                self.count += 1
                if self.count == 1:
                    return first
                exp.stop.set()
                return second
        self.exp.source = SequenceSource()
        self.exp.capture_loop()
        state = self.exp.snapshot()
        self.assertIsNone(state['result'])
        self.assertIn('not visible',state['detection'])
        rows = [json.loads(line) for line in (Path(self.tmp.name)/'observations.jsonl').read_text().splitlines()]
        self.assertTrue(rows[0]['camera_pose_valid'])
        self.assertFalse(rows[-1]['valid'])
        self.assertIn('not visible',rows[-1]['reason'])

    def test_assets_and_path_boundary(self):
        model = json.load(self.get('/api/model'))
        urls = [v['url'] for link in model['links'] for v in link['visuals']]
        self.assertGreater(len(urls),30)
        for url in urls:
            self.assertGreater(len(self.get(url).read()),84)
        self.assertIn(b'A3U',self.get('/').read())
        with self.assertRaises(HTTPError):
            self.get('/../configs/camera_a3u_face.yaml')


if __name__ == '__main__':
    unittest.main()
