"""Transforms T_A_B map B coordinates into A; metres and right-handed axes."""
from pathlib import Path
import hashlib
import math
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / 'assets/robot_model/a3_t3d0/urdf/model.urdf'
CHAIN = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
         'head_yaw_joint', 'head_pitch_joint')
OPTICAL = np.array([[0., 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    out = np.eye(4)
    out[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                   [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    out[:3, 3] = xyz
    return out


def rigid(value):
    out = np.asarray(value, dtype=float)
    if (out.shape != (4, 4) or not np.isfinite(out).all()
            or not np.allclose(out[3], [0, 0, 0, 1])
            or not np.allclose(out[:3, :3].T @ out[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(out[:3, :3]), 1)):
        raise ValueError('Expected a finite right-handed rigid 4x4 transform')
    return out


def load_config(path):
    cfg = yaml.safe_load(Path(path).read_text())
    if not cfg.get('calibrated') or not cfg.get('calibration_source'):
        raise ValueError('Real input requires offline calibrated intrinsics; use calibrate subcommand or --source demo')
    if str(cfg['calibration_source']).upper().startswith('SYNTHETIC'):
        raise ValueError('Synthetic demo intrinsics are forbidden for real/image inputs; calibrate the selected physical lens')
    k = np.asarray(cfg['camera_matrix'], float)
    d = np.asarray(cfg['distortion_coefficients'], float).reshape(-1)
    size = cfg['image_size']
    if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or d.size not in (4, 5, 8, 12, 14)
            or not np.isfinite(d).all() or len(size) != 2 or min(size) <= 0):
        raise ValueError('Invalid intrinsics, image_size or OpenCV distortion coefficients')
    if cfg.get('distortion_model') not in ('plumb_bob', 'rational_polynomial', 'fisheye'):
        raise ValueError('Supported distortion models: plumb_bob, rational_polynomial, fisheye (OpenCV four coefficients)')
    if cfg['distortion_model'] == 'fisheye' and d.size != 4:
        raise ValueError('OpenCV fisheye requires exactly four coefficients; do not concatenate vendor k/p fields')
    rigid(cfg['T_mount_optical'])
    if cfg.get('optical_parent_link') is not None:
        rigid(cfg['T_parent_optical'])
        if not cfg.get('urdf_sha256'):
            raise ValueError('Direct optical extrinsics require a pinned URDF SHA256')
        if (cfg.get('lens_translation_mount_m') is not None
                or np.any(cfg['delta_mount_xyz_m']) or np.any(cfg['delta_mount_rpy_rad'])
                or not np.allclose(cfg['T_mount_optical'], OPTICAL)):
            raise ValueError('Direct parent extrinsics cannot be combined with CAD mount corrections')
    for key in ('delta_mount_xyz_m', 'delta_mount_rpy_rad'):
        v = np.asarray(cfg[key], float)
        if v.shape != (3,) or not np.isfinite(v).all():
            raise ValueError('Invalid ' + key)
    lens = cfg.get('lens_translation_mount_m')
    if lens is not None and (np.asarray(lens).shape != (3,) or not np.isfinite(lens).all()):
        raise ValueError('Invalid lens translation')
    for key in ('length_m', 'width_m', 'height_m', 'net_height_m'):
        if not math.isfinite(cfg['table'][key]) or cfg['table'][key] <= 0:
            raise ValueError('Invalid table dimension: ' + key)
    if cfg['marker']['size_m'] <= 0:
        raise ValueError('Marker size must be positive')
    return cfg


def vector(node, attr, default):
    return [float(v) for v in (node.get(attr, default) if node is not None else default).split()]


class RobotModel:
    def __init__(self, path=DEFAULT_URDF):
        self.path = Path(path).resolve()
        self.sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        tree = ET.parse(self.path).getroot()
        self.joints = []
        self.links = []
        self.meshes = {}
        for node in tree.findall('joint'):
            limit = node.find('limit')
            self.joints.append(dict(name=node.get('name'), type=node.get('type'),
                parent=node.find('parent').get('link'), child=node.find('child').get('link'),
                xyz=vector(node.find('origin'), 'xyz', '0 0 0'),
                rpy=vector(node.find('origin'), 'rpy', '0 0 0'),
                axis=vector(node.find('axis'), 'xyz', '1 0 0'),
                lower=float(limit.get('lower', '-inf')) if limit is not None else -math.inf,
                upper=float(limit.get('upper', 'inf')) if limit is not None else math.inf))
        self.by_name = {j['name']: j for j in self.joints}
        for name in CHAIN:
            if name not in self.by_name:
                raise ValueError('Missing FK joint: ' + name)
        # Resolve the actual CAD camera node, not a hard-coded optical frame.
        cams = [j for j in self.joints if j['child'].endswith('head_stereo_camera_Link')]
        if len(cams) != 1:
            raise ValueError('Expected one head stereo mount')
        self.camera_link = cams[0]['child']
        for link in tree.findall('link'):
            visuals = []
            for vis in link.findall('visual'):
                mesh = vis.find('geometry/mesh')
                if mesh is None:
                    continue
                filename = mesh.get('filename')
                if filename.startswith('package://'):
                    raise ValueError('Use a URDF with relative mesh paths')
                path = (self.path.parent / filename).resolve()
                key = str(len(self.meshes))
                self.meshes[key] = path
                visuals.append(dict(url='/mesh/' + key, xyz=vector(vis.find('origin'), 'xyz', '0 0 0'),
                    rpy=vector(vis.find('origin'), 'rpy', '0 0 0'), scale=vector(mesh, 'scale', '1 1 1')))
            self.links.append(dict(name=link.get('name'), visuals=visuals))

    def fk(self, positions, strict=True):
        if strict:
            for name in CHAIN:
                q = positions.get(name)
                j = self.by_name[name]
                if q is None or not math.isfinite(q) or not j['lower'] <= q <= j['upper']:
                    raise ValueError('Missing, invalid or out-of-limits joint: ' + name)
        frames = {'pelvis_link': np.eye(4)}
        pending = list(self.joints)
        while pending:
            progress = False
            for j in pending[:]:
                if j['parent'] not in frames:
                    continue
                t = transform(j['xyz'], j['rpy'])
                q = positions.get(j['name'], 0.)
                if j['type'] != 'fixed':
                    axis = np.asarray(j['axis'], float)
                    axis /= np.linalg.norm(axis)
                    motion = np.eye(4)
                    if j['type'] in ('revolute', 'continuous'):
                        motion[:3, :3] = cv2.Rodrigues(axis * q)[0]
                    elif j['type'] == 'prismatic':
                        motion[:3, 3] = axis * q
                    else:
                        raise ValueError('Unsupported joint type')
                    t = t @ motion
                frames[j['child']] = frames[j['parent']] @ t
                pending.remove(j)
                progress = True
            if not progress:
                raise ValueError('URDF has disconnected links or non-pelvis root')
        return frames

    def optical_pose(self, positions, cfg):
        frames = self.fk(positions)
        if cfg.get('optical_parent_link') is not None:
            self.validate_camera_config(cfg)
            return frames[cfg['optical_parent_link']] @ rigid(cfg['T_parent_optical'])
        mount = frames[self.camera_link]
        optical = rigid(cfg['T_mount_optical']).copy()
        if cfg.get('lens_translation_mount_m') is not None:
            optical[:3, 3] += cfg['lens_translation_mount_m']
        return mount @ transform(cfg['delta_mount_xyz_m'], cfg['delta_mount_rpy_rad']) @ optical

    def validate_camera_config(self, cfg):
        if cfg.get('optical_parent_link') is not None:
            if cfg['optical_parent_link'] not in {link['name'] for link in self.links}:
                raise ValueError('Optical parent link absent from selected URDF')
            if self.sha256 != cfg['urdf_sha256']:
                raise ValueError('Camera extrinsics belong to a different URDF; use the pinned model')


def table_corners(cfg):
    x, y = cfg['table']['length_m']/2, cfg['table']['width_m']/2
    # Near-left, near-right, far-right, far-left in TABLE coordinates.
    return np.array([[-x, y, 0], [-x, -y, 0], [x, -y, 0], [x, y, 0]], float)


def table_edges(cfg):
    """Sample straight 3D edges before nonlinear image projection."""
    corners = table_corners(cfg)
    a = np.linspace(0, 1, 17, endpoint=False)[:, None]
    return np.concatenate([(1-a)*p + a*q for p, q in zip(corners, np.roll(corners, -1, axis=0))])


def marker_points(cfg):
    marker = cfg['marker']
    s = marker['size_m']/2
    # Detector order TL, TR, BR, BL. Top direction = table +x at yaw=0.
    points = np.array([[s, s, 0], [s, -s, 0], [-s, -s, 0], [-s, s, 0]])
    return points @ transform(rpy=(0, 0, marker['yaw_rad']))[:3, :3].T + marker['center_table_m']


def project(points, t, cfg):
    if cfg.get('distortion_model') == 'fisheye':
        return cv2.fisheye.projectPoints(np.asarray(points, float).reshape(-1, 1, 3),
            cv2.Rodrigues(t[:3, :3])[0], t[:3, 3], np.asarray(cfg['camera_matrix'], float),
            np.asarray(cfg['distortion_coefficients'], float).reshape(4, 1))[0].reshape(-1, 2)
    return cv2.projectPoints(np.asarray(points, float), cv2.Rodrigues(t[:3, :3])[0], t[:3, 3],
        np.asarray(cfg['camera_matrix'], float), np.asarray(cfg['distortion_coefficients'], float))[0].reshape(-1, 2)


def solve_pose(objects, pixels, cfg):
    objects, pixels = np.asarray(objects, float), np.asarray(pixels, float)
    if (objects.shape != (4, 3) or pixels.shape != (4, 2)
            or not np.isfinite(objects).all() or not np.isfinite(pixels).all()):
        raise ValueError('Four finite corresponding points required')
    contour = pixels.astype(np.float32)
    if not cv2.isContourConvex(contour) or abs(cv2.contourArea(contour)) < cfg['quality']['min_area_px']:
        raise ValueError('Corners are crossed, collinear or too small')
    w, h = cfg['image_size']
    if np.any(pixels < 0) or np.any(pixels >= [w, h]):
        raise ValueError('Corner outside calibrated image')
    k = np.asarray(cfg['camera_matrix'], float)
    d = np.asarray(cfg['distortion_coefficients'], float)
    solve_pixels = pixels
    if cfg.get('distortion_model') == 'fisheye':
        solve_pixels = cv2.fisheye.undistortPoints(pixels.reshape(-1, 1, 2), k, d.reshape(4, 1)).reshape(-1, 2)
        if not np.isfinite(solve_pixels).all():
            raise ValueError('Fisheye inverse projection failed')
        rays = np.column_stack((solve_pixels, np.ones(len(pixels))))
        if np.max(np.linalg.norm(project(rays, np.eye(4), cfg)-pixels, axis=1)) > .05:
            raise ValueError('Fisheye inverse projection did not converge in the forward hemisphere')
        k, d = np.eye(3), None
    output = cv2.solvePnPGeneric(objects, solve_pixels, k, d, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for rv, tv in zip(output[1], output[2]):
        rv, tv = cv2.solvePnPRefineLM(objects, solve_pixels, k, d, rv.copy(), tv.copy())
        t = np.eye(4)
        t[:3, :3], t[:3, 3] = cv2.Rodrigues(rv)[0], tv.reshape(3)
        optical_in_table = np.linalg.inv(t)
        depth = (objects @ t[:3, :3].T + t[:3, 3])[:, 2]
        error = float(np.sqrt(np.mean(np.sum((project(objects, t, cfg)-pixels)**2, axis=1))))
        # Above the tabletop, all corners in front, bounded distance, low reprojection error.
        if (np.min(depth) > 0 and optical_in_table[2, 3] > 0
                and np.linalg.norm(tv) <= cfg['quality']['max_distance_m']
                and error <= cfg['quality']['max_reprojection_px']):
            if not any(np.allclose(t, old[1], atol=1e-5) for old in candidates):
                candidates.append((error, t))
    if not candidates:
        raise ValueError('No physically valid low-error planar pose')
    candidates.sort(key=lambda v: v[0])
    ambiguous = len(candidates) > 1 and candidates[1][0]-candidates[0][0] < cfg['quality']['ambiguity_px']
    return candidates[0][1], dict(reprojection_rms_px=candidates[0][0],
        ambiguous=ambiguous, candidate_errors_px=[p[0] for p in candidates])


def metrics(t_table_pelvis, cfg):
    p = t_table_pelvis[:3, 3]
    x, y = cfg['table']['length_m']/2, cfg['table']['width_m']/2
    return dict(pelvis_table_xyz_m=p.tolist(),
        pelvis_to_table_center_3d_m=float(np.linalg.norm(p)),
        pelvis_to_table_center_xy_m=float(np.linalg.norm(p[:2])),
        pelvis_to_table_rectangle_xy_m=float(np.linalg.norm(np.maximum(np.abs(p[:2])-[x, y], 0))),
        near_end_signed_gap_x_m=float(-x-p[0]),
        robot_heading_table_rad=math.atan2(t_table_pelvis[1, 0], t_table_pelvis[0, 0]))


def interpolate_joints(samples, stamp_ns, max_gap_ns=100_000_000):
    """Require exact capture stamp or bracketing samples. Never extrapolate/hold."""
    for t, q in samples:
        if t == stamp_ns:
            return dict(q)
    for (ta, qa), (tb, qb) in zip(samples, samples[1:]):
        if ta < stamp_ns < tb and tb-ta <= max_gap_ns:
            if any(n not in qa or n not in qb for n in CHAIN):
                break
            a = (stamp_ns-ta)/(tb-ta)
            return {n: qa[n]+a*(qb[n]-qa[n]) for n in CHAIN}
    raise ValueError('No valid joint bracket at image capture timestamp (gap <=100 ms required)')
