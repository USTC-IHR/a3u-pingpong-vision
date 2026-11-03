"""Fisheye geometry and factory gates. Synthetic evidence, not field acceptance."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

from face_table.app import Experiment
from face_table.factory import import_factory
from face_table.geometry import (ROOT, CHAIN, RobotModel, load_config, project,
                                 solve_pose, table_corners, transform)
from face_table.sources import Frame


def analytic_pixels(objects, pose, cfg):
    """Independent four-term theta polynomial, not the production projector."""
    p = objects @ pose[:3, :3].T + pose[:3, 3]
    xy = p[:, :2]/p[:, 2:]
    r = np.linalg.norm(xy, axis=1)
    theta = np.arctan(r)
    scale = theta * (1 + sum(k*theta**(2*(i+1)) for i, k in enumerate(cfg['distortion_coefficients']))) / r
    k = np.array(cfg['camera_matrix'])
    return xy*scale[:, None]*[k[0, 0], k[1, 1]] + k[:2, 2]


class FisheyeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.path = Path(cls.tmp.name)/'factory.yaml'
        cls.urdf = ROOT/'assets/robot_model/a3_ultra_t2d5/urdf/model.urdf'
        cls.args = SimpleNamespace(sensor=ROOT/'robot_param/sensor_param.yaml',
            hal=ROOT/'robot_param/hal/rgb_camera_params.yaml', urdf=cls.urdf, output=cls.path)
        import_factory(cls.args)
        cls.cfg = load_config(cls.path)
        cls.model = RobotModel(cls.urdf)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_independent_theta_projection_and_pose_recovery(self):
        objects = table_corners(self.cfg)
        for yaw, pitch, x in [(0., .15, -2.2), (.15, -.08, -2.8), (-.2, .22, -3.5)]:
            q = dict.fromkeys(CHAIN, 0.)
            q.update(head_yaw_joint=yaw, head_pitch_joint=pitch)
            optical = self.model.optical_pose(q, self.cfg)
            truth = transform([x, .13, .35], [.01, -.02, .04])
            camera_table = np.linalg.inv(truth @ optical)
            pixels = analytic_pixels(objects, camera_table, self.cfg)
            np.testing.assert_allclose(project(objects, camera_table, self.cfg), pixels, atol=1e-9)
            resolved, quality = solve_pose(objects, pixels, self.cfg)
            np.testing.assert_allclose(np.linalg.inv(resolved) @ np.linalg.inv(optical), truth, atol=1e-6)
            self.assertFalse(quality['ambiguous'])
            self.assertLess(quality['reprojection_rms_px'], 1e-6)

    def test_subpixel_noise_stays_bounded(self):
        q = dict.fromkeys(CHAIN, 0.)
        truth = transform([-2.6, .12, .4])
        optical = self.model.optical_pose(q, self.cfg)
        pixels = analytic_pixels(table_corners(self.cfg), np.linalg.inv(truth @ optical), self.cfg)
        rng = np.random.default_rng(42)
        for _ in range(20):
            noisy = pixels + rng.normal(0, .25, pixels.shape)
            resolved, quality = solve_pose(table_corners(self.cfg), noisy, self.cfg)
            recovered = np.linalg.inv(resolved) @ np.linalg.inv(optical)
            self.assertLess(np.linalg.norm(recovered[:3, 3]-truth[:3, 3]), .04)
            self.assertLess(quality['reprojection_rms_px'], 1.)

    def test_direct_transform_has_no_duplicate_cad_mount(self):
        q = dict.fromkeys(CHAIN, 0.)
        ref = self.model.fk(q)['head_pitch_Link']
        expected = ref @ np.array(self.cfg['T_parent_optical'])
        np.testing.assert_allclose(self.model.optical_pose(q, self.cfg), expected)
        self.assertFalse(self.cfg['optical_extrinsic_verified'])
        self.assertFalse(self.cfg['factory_reference_mapping_verified'])
        with self.assertRaisesRegex(ValueError, 'different URDF'):
            RobotModel().validate_camera_config(self.cfg)

    def test_coefficients_and_mixed_extrinsics_fail_closed(self):
        for key, value in [('distortion_coefficients', [0]*8),
                           ('delta_mount_xyz_m', [.01, 0, 0]),
                           ('lens_translation_mount_m', [0, 0, 0])]:
            cfg = copy.deepcopy(self.cfg)
            cfg[key] = value
            path = Path(self.tmp.name)/'bad.yaml'
            path.write_text(yaml.safe_dump(cfg))
            with self.assertRaises(ValueError):
                load_config(path)

    def test_mismatched_hal_is_not_imported(self):
        data = yaml.safe_load(Path(self.args.hal).read_text())
        entry = next(v for v in data.values() if v['name'] == 'front_left_camera')
        entry['coeff'][0] += .1
        path = Path(self.tmp.name)/'bad-hal.yaml'
        path.write_text(yaml.safe_dump(data))
        args = copy.copy(self.args)
        args.hal = path
        with self.assertRaisesRegex(ValueError, 'disagree'):
            import_factory(args)

    def test_camera_solution_does_not_claim_verified_robot(self):
        q = dict.fromkeys(CHAIN, 0.)
        args = SimpleNamespace(assume_zero_joints=False, joints=None, source='image',
            output=str(Path(self.tmp.name)/'experiment'), image_topic=None, joint_topic=None,
            device=None, preview=False)
        exp = Experiment(self.cfg, self.model, None, args)
        optical = self.model.optical_pose(q, self.cfg)
        truth = transform([-2.6, .12, .4])
        pixels = analytic_pixels(table_corners(self.cfg), np.linalg.inv(truth @ optical), self.cfg)
        item = dict(id=1, frame=Frame(None, positions=q, joint_status='synchronized_test'), jpeg=b'test')
        try:
            result = exp.compute(item, table_corners(self.cfg), pixels, 'synthetic_test')
            self.assertTrue(result['camera_pose_valid'])
            self.assertEqual(len(result['table_edges_px']), 68)
            np.testing.assert_allclose(np.array(result['table_edges_px'])[::17], result['table_outline_px'])
            self.assertFalse(result['robot_pose_valid'])
            self.assertEqual(result['robot_pose_grade'], 'factory_unverified')
            self.assertTrue(any('unverified' in w for w in result['warnings']))
            json.dumps(result, allow_nan=False)
        finally:
            exp.log.close()


if __name__ == '__main__':
    unittest.main()
