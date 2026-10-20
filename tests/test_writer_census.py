"""CSV schema and live-view aggregation — both pure, no file or radio needed."""

import csv
import io
import unittest

from ble_census.ble_scan import Advertisement
from ble_census.census import Aggregator
from ble_census.writer import FIELDNAMES, CensusWriter, to_row


def _adv(addr="AA:BB:CC:DD:EE:FF", **kw):
    base = dict(addr=addr, addr_type="public", rssi=-60.0)
    base.update(kw)
    return Advertisement(**base)


class TestToRow(unittest.TestCase):
    def test_pose_blank_off_robot(self):
        row = to_row(_adv())
        self.assertEqual(row["x"], "")  # blank, never 0 — 0 would plot at origin
        self.assertEqual(row["stamp"], "")

    def test_pose_filled_on_robot(self):
        pose = {"stamp": 1760932481.2, "sample_id": 42, "x": 1.0, "y": 2.0, "z": 0.4, "yaw": 1.57}
        row = to_row(_adv(), pose)
        self.assertEqual(row["sample_id"], 42)
        self.assertEqual(row["x"], 1.0)

    def test_company_id_hex(self):
        self.assertEqual(to_row(_adv(company_id=0x004C))["company_id"], "0x004c")

    def test_company_id_blank_when_absent(self):
        self.assertEqual(to_row(_adv())["company_id"], "")

    def test_tx_power_blank_when_none(self):
        self.assertEqual(to_row(_adv())["tx_power"], "")
        self.assertEqual(to_row(_adv(tx_power=-59))["tx_power"], -59)

    def test_name_blank_when_none(self):
        self.assertEqual(to_row(_adv())["name"], "")

    def test_row_has_exactly_the_schema_keys(self):
        self.assertEqual(set(to_row(_adv()).keys()), set(FIELDNAMES))

    def test_writer_roundtrips_through_csv(self):
        buf = io.StringIO()
        w = CensusWriter(buf)
        w.write(_adv(addr="11:22:33:44:55:66", category="tracker", adv_type="findmy"))
        buf.seek(0)
        rows = list(csv.DictReader(buf))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["addr"], "11:22:33:44:55:66")
        self.assertEqual(rows[0]["category"], "tracker")


class TestAggregator(unittest.TestCase):
    def test_collapses_by_address_and_counts(self):
        agg = Aggregator()
        agg.add(_adv(rssi=-70))
        agg.add(_adv(rssi=-55))  # same addr, stronger
        self.assertEqual(len(agg), 1)
        self.assertEqual(agg.summaries()[0].count, 2)
        self.assertEqual(agg.summaries()[0].best_rssi, -55)  # keeps the nearest

    def test_later_packet_fills_missing_category(self):
        agg = Aggregator()
        agg.add(_adv(category="unknown"))
        agg.add(_adv(category="tracker"))
        self.assertEqual(agg.summaries()[0].category, "tracker")

    def test_sorted_strongest_first(self):
        agg = Aggregator()
        agg.add(_adv(addr="AA:00:00:00:00:01", rssi=-80))
        agg.add(_adv(addr="AA:00:00:00:00:02", rssi=-40))
        self.assertEqual(agg.summaries()[0].addr, "AA:00:00:00:00:02")

    def test_category_counts(self):
        agg = Aggregator()
        agg.add(_adv(addr="AA:00:00:00:00:01", category="tracker"))
        agg.add(_adv(addr="AA:00:00:00:00:02", category="tracker"))
        agg.add(_adv(addr="AA:00:00:00:00:03", category="phone"))
        self.assertEqual(agg.category_counts(), {"tracker": 2, "phone": 1})

    def test_persistent_fraction(self):
        agg = Aggregator()
        agg.add(_adv(addr="AA:00:00:00:00:01", addr_type="public", privacy="public"))
        agg.add(_adv(addr="7F:00:00:00:00:02", addr_type="random", privacy="resolvable-random"))
        self.assertAlmostEqual(agg.persistent_fraction(), 0.5)

    def test_persistent_fraction_empty(self):
        self.assertEqual(Aggregator().persistent_fraction(), 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
