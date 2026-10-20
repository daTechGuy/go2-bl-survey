"""The parsed-fields path must agree with the raw-bytes path.

A backend like bleak hands us the advertisement already split into dicts; the
raw sniffer hands us bytes. Both must classify a device identically — that is
the whole reason `advertisement_from_parsed` reuses the same sub-decoders.
"""

import unittest

from ble_census.ble_scan import advertisement_from_parsed


class TestFromParsed(unittest.TestCase):
    def test_find_my_tracker(self):
        adv = advertisement_from_parsed(
            "7F:11:22:33:44:55", "random", -70,
            manufacturer_data={0x004C: b"\x12\x19" + b"\x00" * 25},
        )
        self.assertEqual(adv.adv_type, "findmy")
        self.assertEqual(adv.category, "tracker")

    def test_airpods_wearable(self):
        adv = advertisement_from_parsed(
            "7F:11:22:33:44:55", "random", -55,
            manufacturer_data={0x004C: b"\x07\x19" + b"\x01" * 25},
        )
        self.assertEqual(adv.category, "wearable")

    def test_ibeacon_beacon(self):
        uuid = bytes.fromhex("e2c56db5dffb48d2b060d0f5a71096e0")
        body = b"\x02\x15" + uuid + (1).to_bytes(2, "big") + (2).to_bytes(2, "big") + b"\xc5"
        adv = advertisement_from_parsed(
            "AA:BB:CC:DD:EE:FF", "public", -65, manufacturer_data={0x004C: body},
        )
        self.assertEqual(adv.adv_type, "ibeacon")
        self.assertEqual(adv.category, "beacon")

    def test_eddystone_url_beacon(self):
        adv = advertisement_from_parsed(
            "AA:BB:CC:DD:EE:FF", "public", -80,
            service_data={0xFEAA: b"\x10\xee\x03example\x07"},
        )
        self.assertEqual(adv.category, "beacon")
        self.assertEqual(adv.detail["eddystone"]["url"], "https://example.com")

    def test_tile_tracker(self):
        adv = advertisement_from_parsed(
            "AA:BB:CC:DD:EE:FF", "public", -72, service_uuids=[0xFEED],
        )
        self.assertEqual(adv.category, "tracker")

    def test_unknown_addr_type_is_unknown_privacy(self):
        # Windows/macOS backends don't report address type — don't guess.
        adv = advertisement_from_parsed("7F:11:22:33:44:55", "", -60)
        self.assertEqual(adv.privacy, "unknown")
        self.assertFalse(adv.persistent)

    def test_resolvable_phone(self):
        adv = advertisement_from_parsed(
            "7F:AA:BB:CC:DD:EE", "random", -58,
            name="Pixel Buds", manufacturer_data={0x00E0: b"\x01\x02"},
        )
        self.assertEqual(adv.manufacturer, "Google")
        self.assertEqual(adv.privacy, "resolvable-random")
        self.assertEqual(adv.category, "phone")

    def test_empty_advert(self):
        adv = advertisement_from_parsed("AA:BB:CC:DD:EE:FF", "public", -90)
        self.assertEqual(adv.category, "unknown")
        self.assertEqual(adv.adv_type, "plain")


if __name__ == "__main__":
    unittest.main(verbosity=2)
