"""Device localization from geotagged sightings."""

import math
import unittest

from ble_census.locate import locate, locate_rows


class TestLocate(unittest.TestCase):
    def test_no_sightings(self):
        self.assertIsNone(locate([]))

    def test_single_sighting_is_that_point(self):
        est = locate([(3.0, 4.0, -60.0)])
        self.assertAlmostEqual(est["x"], 3.0)
        self.assertAlmostEqual(est["y"], 4.0)
        self.assertEqual(est["n"], 1)
        self.assertEqual(est["best_rssi"], -60.0)

    def test_weighted_toward_strongest(self):
        # equal geometry, but one sighting is much stronger -> estimate pulled to it
        est = locate([(0.0, 0.0, -90.0), (10.0, 0.0, -50.0)])
        self.assertGreater(est["x"], 9.0)  # -50 dBm dominates -90 by ~10000x

    def test_ring_around_a_point_recovers_center(self):
        # device at (5,5); sample on a circle of radius 3, RSSI falls with
        # distance per a log-distance model. Estimate should land near centre.
        dev = (5.0, 5.0)
        sightings = []
        for deg in range(0, 360, 30):
            x = dev[0] + 3.0 * math.cos(math.radians(deg))
            y = dev[1] + 3.0 * math.sin(math.radians(deg))
            # plus one close pass
            sightings.append((x, y, -70.0))
        sightings.append((5.2, 5.1, -48.0))  # near approach
        est = locate(sightings)
        self.assertLess(math.hypot(est["x"] - 5.0, est["y"] - 5.0), 1.0)


class TestLocateRows(unittest.TestCase):
    def _rows(self, addr, privacy, pts):
        return [
            {"addr": addr, "privacy": privacy, "x": x, "y": y, "rssi": r}
            for x, y, r in pts
        ]

    def test_filters_rotating_addresses(self):
        rows = self._rows("rot", "resolvable-random", [(0, 0, -60)] * 6)
        self.assertEqual(locate_rows(rows), {})  # phones don't get localized

    def test_locates_persistent_device(self):
        rows = self._rows("beacon1", "public", [(1, 1, -55), (1.2, 0.9, -57), (2, 2, -70), (0.9, 1.1, -54)])
        out = locate_rows(rows, min_sightings=4)
        self.assertIn("beacon1", out)
        self.assertEqual(out["beacon1"]["privacy"], "public")

    def test_min_sightings_enforced(self):
        rows = self._rows("beacon1", "public", [(1, 1, -55), (1, 1, -56)])
        self.assertEqual(locate_rows(rows, min_sightings=4), {})

    def test_blank_pose_rows_skipped(self):
        rows = self._rows("beacon1", "public", [(1, 1, -55)] * 3)
        rows += [{"addr": "beacon1", "privacy": "public", "x": "", "y": "", "rssi": -50}] * 3
        out = locate_rows(rows, min_sightings=4)
        # only the 3 real rows count -> below min, nothing located
        self.assertEqual(out, {})

    def test_static_random_is_persistent(self):
        rows = self._rows("s", "static-random", [(1, 1, -55)] * 5)
        self.assertIn("s", locate_rows(rows, min_sightings=4))


if __name__ == "__main__":
    unittest.main(verbosity=2)
