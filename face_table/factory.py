"""Import the ADU front-left snapshot without claiming physical verification."""
import hashlib
from pathlib import Path

import numpy as np
import yaml

from .geometry import ROOT, OPTICAL, RobotModel, rigid, load_config


def import_factory(args):
    sensor_path, hal_path = Path(args.sensor).resolve(), Path(args.hal).resolve()
    urdf = Path(args.urdf).resolve()
    sensor = yaml.safe_load(sensor_path.read_text())['head_stereo_left_fisheye_camera']
    cameras = yaml.safe_load(hal_path.read_text())
    matches = [v for v in cameras.values() if v['name'] == 'front_left_camera']
    if len(matches) != 1:
        raise ValueError('Expected exactly one HAL front_left_camera; never bind by duplicated SN')
    hal = matches[0]
    if sensor['model_type'] != 'KANNALA_BRANDT' or sensor['ref_frame'] != 'ultra_head_pitch_Link':
        raise ValueError('Unexpected factory model or reference frame')
    intrinsics = [sensor['K'][k] for k in ('fx', 'fy', 'cx', 'cy')]
    coeffs = [sensor['D'][k] for k in ('k2', 'k3', 'k4', 'k5')]
    if (len(hal['coeff']) != 4 or not np.allclose(intrinsics, hal['intrinsics'])
            or not np.allclose(coeffs, hal['coeff'])
            or [sensor['image_width'], sensor['image_height']] != [hal['width'], hal['height']]):
        raise ValueError('HAL and calibration intrinsics disagree')
    if any(sensor['D'][k] != 0 for k in ('k1', 'k6', 'k7', 'k8', 'p1', 'p2')):
        raise ValueError('Unexpected nonzero vendor coefficients outside the four-term HAL model')
    # Candidate: q/t map optical vectors into the factory reference. The physical
    # direction and equivalence to the URDF reference are NOT yet verified.
    quat = np.array([sensor['q']['q_'+k] for k in ('w', 'x', 'y', 'z')])
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or not np.isclose(norm, 1., atol=1e-5):
        raise ValueError('Invalid factory quaternion')
    w, x, y, z = quat/norm
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = [sensor['t']['t_'+k] for k in ('x', 'y', 'z')]
    rigid(t)
    cfg = yaml.safe_load((ROOT/'configs/camera_a3u_face.yaml').read_text())
    fx, fy, cx, cy = intrinsics
    cfg.update(camera_id='head_stereo_left_fisheye_camera', image_size=[hal['width'], hal['height']],
        camera_matrix=[[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
        distortion_coefficients=hal['coeff'], distortion_model='fisheye', calibrated=True,
        calibration_source='ADU_factory_snapshot_HAL_four_term', intrinsics_field_verified=False,
        optical_extrinsic_verified=False, optical_parent_link='head_pitch_Link', T_parent_optical=t.tolist(),
        T_mount_optical=OPTICAL.tolist(), lens_translation_mount_m=None,
        delta_mount_xyz_m=[0, 0, 0], delta_mount_rpy_rad=[0, 0, 0],
        urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        factory_reference_frame=sensor['ref_frame'], factory_reference_mapping_verified=False,
        factory_transform_convention='candidate_T_reference_optical_q_wxyz_t_m',
        factory_calib_time=yaml.load(sensor_path.read_text(), Loader=yaml.BaseLoader)['head_stereo_left_fisheye_camera']['calib_time'],
        factory_inputs={str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p):
            hashlib.sha256(p.read_bytes()).hexdigest() for p in (sensor_path, hal_path)})
    RobotModel(urdf).validate_camera_config(cfg)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(cfg, sort_keys=False))
    load_config(output)
    print(f'Saved {output}. Factory candidate only: verify raw lens identity, projection and reference-frame mapping in the field.')
