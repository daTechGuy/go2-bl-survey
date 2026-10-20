"""The bleak-mapping helpers in scan.py — testable without bleak or a radio.

scan.py imports bleak lazily (inside run_scan), so importing the module and
exercising uuid16 / advertisement_from_bleak needs no Bluetooth stack. The
(device, adv_data) pair is faked with trivial stand-ins.
"""

import unittest
from types import SimpleNamespace

from ble_census.scan import advertisement_from_bleak, uuid16


class TestUuid16(unittest.TestCase):
    def test_standard_base_uuid(self):
        self.assertEqual(uuid16("0000feaa-0000-1000-8000-00805f9b34fb"), 0xFEAA)

    def test_uppercase(self):
        self.assertEqual(uuid16("0000FEED-0000-1000-8000-00805F9B34FB"), 0xFEED)

    def test_vendor_128bit_has_no_short_form(self):
        self.assertIsNone(uuid16("e2c56db5-dffb-48d2-b060-d0f5a71096e0"))


def _bleak_pair(address, *, rssi, name=None, tx_power=None,
                manufacturer_data=None, service_uuids=None, service_data=None, details=None):
    device = SimpleNamespace(address=address, details=details)
    adv = SimpleNamespace(
        rssi=rssi,
        local_name=name,
        tx_power=tx_power,
        manufacturer_data=manufacturer_data or {},
        service_uuids=service_uuids or [],
        service_data=service_data or {},
    )
    return device, adv


class TestAdvertisementFromBleak(unittest.TestCase):
    def test_eddystone_via_service_data(self):
        device, adv_data = _bleak_pair(
            "AA:BB:CC:DD:EE:FF", rssi=-80,
            service_data={"0000feaa-0000-1000-8000-00805f9b34fb": b"\x10\xee\x03example\x07"},
        )
        adv = advertisement_from_bleak(device, adv_data)
        self.assertEqual(adv.category, "beacon")
        self.assertEqual(adv.detail["eddystone"]["url"], "https://example.com")

    def test_addr_type_from_bluez_details(self):
        device, adv_data = _bleak_pair(
            "7F:11:22:33:44:55", rssi=-60, details={"props": {"AddressType": "random"}},
        )
        adv = advertisement_from_bleak(device, adv_data)
        self.assertEqual(adv.privacy, "resolvable-random")

    def test_missing_details_gives_unknown_privacy(self):
        device, adv_data = _bleak_pair("7F:11:22:33:44:55", rssi=-60)
        adv = advertisement_from_bleak(device, adv_data)
        self.assertEqual(adv.privacy, "unknown")

    def test_vendor_service_uuid_dropped(self):
        device, adv_data = _bleak_pair(
            "AA:BB:CC:DD:EE:FF", rssi=-70,
            service_uuids=["e2c56db5-dffb-48d2-b060-d0f5a71096e0"],  # 128-bit, no short form
        )
        adv = advertisement_from_bleak(device, adv_data)
        self.assertEqual(adv.service_uuids, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
