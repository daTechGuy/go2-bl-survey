"""Pose buffer + lookup — the ROS-free half of the geotag."""

import math
import unittest

from ble_census.posejoin import PoseBuffer, _lerp_angle


class TestLerpAngle(unittest.TestCase):
    def test_midpoint(self):
        self.assertAlmostEqual(_lerp_angle(0.0, 1.0, 0.5), 0.5)

    def test_shortest_arc_across_pi(self):
        # from +170 deg to -170 deg is +20 deg the short way, not -340
        a, b = math.radians(170), math.radians(-170)
        mid = _lerp_angle(a, b, 0.5)
        self.assertAlmostEqual(mid, math.pi, places=5)


class TestPoseBuffer(unittest.TestCase):
    def _buf(self):
        # TF-like cadence: poses 0.4 s apart, within a 0.3 s staleness tolerance
        b = PoseBuffer()
        b.add(100.0, 0.0, 0.0, 0.4, 0.0)
        b.add(100.4, 1.0, 2.0, 0.4, 0.0)
        return b

    def test_empty_returns_none(self):
        self.assertIsNone(PoseBuffer().lookup(100.0, 0.3))

    def test_interpolates_between_bracketing_poses(self):
        pose = self._buf().lookup(100.2, 0.3)  # midway, fresh on both sides
        self.assertAlmostEqual(pose["x"], 0.5)
        self.assertAlmostEqual(pose["y"], 1.0)

    def test_exact_stamp_snaps_to_that_pose(self):
        pose = self._buf().lookup(100.4, 0.3)
        self.assertAlmostEqual(pose["x"], 1.0)

    def test_stale_before_first_sample_discarded(self):
        self.assertIsNone(self._buf().lookup(99.0, 0.3))  # 1 s before, max_age 0.3

    def test_stale_after_last_sample_discarded(self):
        self.assertIsNone(self._buf().lookup(102.0, 0.3))

    def test_nearest_when_just_outside_bracket(self):
        b = PoseBuffer()
        b.add(100.0, 5.0, 5.0, 0.4, 0.0)
        pose = b.lookup(100.2, 0.3)  # only one sample, within age
        self.assertAlmostEqual(pose["x"], 5.0)

    def test_wide_gap_not_interpolated(self):
        # two samples 10 s apart; a query in the middle must not interpolate
        # across a gap far larger than max_age — that pose is unknown
        b = PoseBuffer()
        b.add(100.0, 0.0, 0.0, 0.4, 0.0)
        b.add(110.0, 10.0, 0.0, 0.4, 0.0)
        self.assertIsNone(b.lookup(105.0, 0.3))

    def test_out_of_order_adds_sorted(self):
        b = PoseBuffer()
        b.add(100.4, 1.0, 0.0, 0.4, 0.0)
        b.add(100.0, 0.0, 0.0, 0.4, 0.0)
        pose = b.lookup(100.2, 0.3)
        self.assertAlmostEqual(pose["x"], 0.5)

    def test_max_samples_trims_oldest(self):
        b = PoseBuffer(max_samples=2)
        b.add(100.0, 0.0, 0.0, 0.4, 0.0)
        b.add(101.0, 1.0, 0.0, 0.4, 0.0)
        b.add(102.0, 2.0, 0.0, 0.4, 0.0)
        self.assertEqual(len(b), 2)
        self.assertIsNone(b.lookup(100.0, 0.3))  # oldest gone
        self.assertIsNotNone(b.lookup(102.0, 0.3))

    def test_yaw_interpolated_on_short_arc(self):
        b = PoseBuffer()
        b.add(100.0, 0.0, 0.0, 0.4, math.radians(170))
        b.add(101.0, 0.0, 0.0, 0.4, math.radians(-170))
        pose = b.lookup(100.5, 0.6)
        self.assertAlmostEqual(abs(pose["yaw"]), math.pi, places=5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
