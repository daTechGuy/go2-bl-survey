"""Tests for the BLE advertisement decoder.

Plain stdlib unittest. No radio, no ROS, no network — the whole point of
keeping the decoder pure is that it can be pinned before any hardware exists.
Fixtures are assembled with the small `ad()` / `sd()` helpers so each test
reads as the bytes a controller would actually hand us.

    python run_tests.py        # from the repo root
"""

import unittest

from ble_census.ble_scan import (
    Advertisement,
    address_privacy,
    classify,
    decode_advertisement,
    parse_ad_structures,
    parse_apple_tlvs,
    parse_eddystone,
    parse_ibeacon,
)


def ad(ad_type, data: bytes) -> bytes:
    """One AD structure: [length][type][data]. length = 1 + len(data)."""
    return bytes([1 + len(data), ad_type]) + data


def sd16(uuid16: int, data: bytes) -> bytes:
    """A 16-bit service-data (0x16) AD structure."""
    return ad(0x16, uuid16.to_bytes(2, "little") + data)


def mfr(company_id: int, data: bytes) -> bytes:
    """A manufacturer-specific (0xFF) AD structure."""
    return ad(0xFF, company_id.to_bytes(2, "little") + data)


class TestAdStructureWalker(unittest.TestCase):
    def test_empty_payload(self):
        self.assertEqual(parse_ad_structures(b""), [])

    def test_single_structure(self):
        self.assertEqual(parse_ad_structures(ad(0x09, b"hi")), [(0x09, b"hi")])

    def test_two_structures(self):
        payload = ad(0x01, b"\x06") + ad(0x09, b"Govee")
        self.assertEqual(parse_ad_structures(payload), [(0x01, b"\x06"), (0x09, b"Govee")])

    def test_zero_length_byte_ends_parsing(self):
        payload = ad(0x09, b"ok") + b"\x00\x00\x00"  # trailing zero padding
        self.assertEqual(parse_ad_structures(payload), [(0x09, b"ok")])

    def test_length_overrun_keeps_earlier_structures(self):
        # second structure claims 20 bytes but only a few follow: good one kept
        payload = ad(0x09, b"ok") + bytes([20, 0xFF]) + b"\x01\x02"
        self.assertEqual(parse_ad_structures(payload), [(0x09, b"ok")])

    def test_type_with_no_data(self):
        self.assertEqual(parse_ad_structures(bytes([1, 0x0A])), [(0x0A, b"")])

    def test_garbage_does_not_raise(self):
        for payload in (b"\xff", b"\x02", b"\x10\x09ab", bytes(range(0, 40))):
            parse_ad_structures(payload)  # must not raise


class TestScalarFields(unittest.TestCase):
    def test_complete_name(self):
        adv = decode_advertisement(ad(0x09, b"Govee_H5075"), "AA:BB:CC:DD:EE:FF", "public", -60)
        self.assertEqual(adv.name, "Govee_H5075")

    def test_shortened_name(self):
        adv = decode_advertisement(ad(0x08, b"Gov"), "AA:BB:CC:DD:EE:FF", "public", -60)
        self.assertEqual(adv.name, "Gov")

    def test_name_with_invalid_utf8_does_not_raise(self):
        adv = decode_advertisement(ad(0x09, b"bad\xff\xfe"), "AA:BB:CC:DD:EE:FF", "public", -60)
        self.assertTrue(adv.name.startswith("bad"))

    def test_tx_power_negative(self):
        adv = decode_advertisement(ad(0x0A, b"\xc5"), "AA:BB:CC:DD:EE:FF", "public", -60)
        self.assertEqual(adv.tx_power, -59)

    def test_tx_power_positive(self):
        adv = decode_advertisement(ad(0x0A, b"\x04"), "AA:BB:CC:DD:EE:FF", "public", -60)
        self.assertEqual(adv.tx_power, 4)

    def test_service_uuid_list(self):
        payload = ad(0x03, b"\x0d\x18\x0f\x18")  # 0x180D heart rate, 0x180F battery
        adv = decode_advertisement(payload, "AA:BB:CC:DD:EE:FF", "public", -60)
        self.assertIn(0x180D, adv.service_uuids)
        self.assertIn(0x180F, adv.service_uuids)

    def test_odd_trailing_uuid_byte_ignored(self):
        adv = decode_advertisement(ad(0x03, b"\x0d\x18\x0f"), "AA:BB:CC:DD:EE:FF", "public", -60)
        self.assertEqual(adv.service_uuids, [0x180D])

    def test_rssi_is_float(self):
        adv = decode_advertisement(b"", "AA:BB:CC:DD:EE:FF", "public", -71)
        self.assertEqual(adv.rssi, -71.0)
        self.assertIsInstance(adv.rssi, float)


class TestManufacturer(unittest.TestCase):
    def test_known_company_id(self):
        adv = decode_advertisement(mfr(0x004C, b"\x10\x02\x00\x00"), "AA:BB:CC:DD:EE:FF", "random", -50)
        self.assertEqual(adv.company_id, 0x004C)
        self.assertEqual(adv.manufacturer, "Apple")

    def test_unknown_company_id_is_recorded_not_labelled(self):
        adv = decode_advertisement(mfr(0x1234, b"\x01"), "AA:BB:CC:DD:EE:FF", "public", -50)
        self.assertEqual(adv.company_id, 0x1234)
        self.assertIsNone(adv.manufacturer)
        self.assertEqual(adv.adv_type, "mfr")

    def test_short_manufacturer_data_ignored(self):
        adv = decode_advertisement(ad(0xFF, b"\x4c"), "AA:BB:CC:DD:EE:FF", "public", -50)
        self.assertIsNone(adv.company_id)


class TestIBeacon(unittest.TestCase):
    def _ibeacon_bytes(self):
        uuid = bytes.fromhex("e2c56db5dffb48d2b060d0f5a71096e0")
        return b"\x02\x15" + uuid + (1000).to_bytes(2, "big") + (7).to_bytes(2, "big") + b"\xc5"

    def test_parse_ibeacon(self):
        info = parse_ibeacon(self._ibeacon_bytes())
        self.assertEqual(info["major"], 1000)
        self.assertEqual(info["minor"], 7)
        self.assertEqual(info["measured_power"], -59)
        self.assertEqual(info["uuid"], "e2c56db5-dffb-48d2-b060-d0f5a71096e0")

    def test_not_ibeacon(self):
        self.assertIsNone(parse_ibeacon(b"\x10\x05abc"))

    def test_decode_ibeacon_is_beacon(self):
        adv = decode_advertisement(mfr(0x004C, self._ibeacon_bytes()), "AA:BB:CC:DD:EE:FF", "public", -65)
        self.assertEqual(adv.adv_type, "ibeacon")
        self.assertEqual(adv.category, "beacon")
        self.assertEqual(adv.detail["ibeacon"]["major"], 1000)


class TestApple(unittest.TestCase):
    def test_find_my_is_tracker(self):
        # Apple TLV type 0x12 (offline finding) — a separated AirTag
        payload = mfr(0x004C, b"\x12\x19" + b"\x00" * 25)
        adv = decode_advertisement(payload, "7F:11:22:33:44:55", "random", -70)
        self.assertEqual(adv.adv_type, "findmy")
        self.assertEqual(adv.category, "tracker")

    def test_proximity_pairing_is_wearable(self):
        # Apple TLV type 0x07 — AirPods and kin
        payload = mfr(0x004C, b"\x07\x19" + b"\x01" * 25)
        adv = decode_advertisement(payload, "7F:11:22:33:44:55", "random", -55)
        self.assertEqual(adv.category, "wearable")

    def test_nearby_iphone_is_phone(self):
        # Apple TLV type 0x10 — a nearby, unlocked device
        payload = mfr(0x004C, b"\x10\x06\x00\x00\x00\x00\x00\x00")
        adv = decode_advertisement(payload, "7F:11:22:33:44:55", "random", -40)
        self.assertEqual(adv.category, "phone")

    def test_parse_apple_tlvs_stops_on_overrun(self):
        tlvs = parse_apple_tlvs(b"\x10\x02\xaa\xbb\x12\x40\xcc")  # last claims 0x40 bytes
        self.assertEqual(tlvs, [(0x10, b"\xaa\xbb")])


class TestEddystone(unittest.TestCase):
    def test_url_frame(self):
        # frame 0x10, tx -18, scheme 0x03 'https://', 'example', 0x07 '.com'
        service_data = b"\x10\xee\x03example\x07"
        adv = decode_advertisement(sd16(0xFEAA, service_data), "AA:BB:CC:DD:EE:FF", "public", -80)
        self.assertEqual(adv.adv_type, "eddystone")
        self.assertEqual(adv.category, "beacon")
        self.assertEqual(adv.detail["eddystone"]["url"], "https://example.com")

    def test_uid_frame(self):
        service_data = b"\x00\xee" + b"\xaa" * 10 + b"\xbb" * 6
        info = parse_eddystone(service_data)
        self.assertEqual(info["frame"], "uid")
        self.assertEqual(info["namespace"], "aa" * 10)

    def test_tlm_frame(self):
        service_data = b"\x20\x00" + (3300).to_bytes(2, "big") + b"\x00" * 10
        info = parse_eddystone(service_data)
        self.assertEqual(info["frame"], "tlm")
        self.assertEqual(info["battery_mv"], 3300)

    def test_empty_service_data(self):
        self.assertIsNone(parse_eddystone(b""))


class TestTile(unittest.TestCase):
    def test_tile_service_uuid_is_tracker(self):
        adv = decode_advertisement(ad(0x03, b"\xed\xfe"), "AA:BB:CC:DD:EE:FF", "public", -72)
        self.assertEqual(adv.category, "tracker")


class TestAddressPrivacy(unittest.TestCase):
    def test_public(self):
        self.assertEqual(address_privacy("3C:37:86:A1:B2:C3", "public"), "public")

    def test_static_random(self):  # top two bits 0b11
        self.assertEqual(address_privacy("D1:22:33:44:55:66", "random"), "static-random")

    def test_resolvable_random(self):  # top two bits 0b01
        self.assertEqual(address_privacy("7F:22:33:44:55:66", "random"), "resolvable-random")

    def test_non_resolvable_random(self):  # top two bits 0b00
        self.assertEqual(address_privacy("1A:22:33:44:55:66", "random"), "non-resolvable-random")

    def test_malformed_address(self):
        self.assertEqual(address_privacy("", "random"), "unknown")

    def test_persistent_property(self):
        pub = decode_advertisement(b"", "3C:37:86:A1:B2:C3", "public", -60)
        rpa = decode_advertisement(b"", "7F:22:33:44:55:66", "random", -60)
        self.assertTrue(pub.persistent)
        self.assertFalse(rpa.persistent)


class TestClassifyAndResilience(unittest.TestCase):
    def test_unknown_when_no_signal(self):
        adv = decode_advertisement(ad(0x09, b"MysteryThing"), "11:22:33:44:55:66", "public", -60)
        self.assertEqual(adv.category, "unknown")

    def test_garmin_is_wearable(self):
        adv = Advertisement(addr="x", addr_type="public", rssi=-60, company_id=0x0087)
        self.assertEqual(classify(adv), "wearable")

    def test_heart_rate_is_wearable(self):
        adv = Advertisement(addr="x", addr_type="public", rssi=-60, service_uuids=[0x180D])
        self.assertEqual(classify(adv), "wearable")

    def test_full_realistic_packet_does_not_raise(self):
        # flags + name + tx power + services + manufacturer, back to back
        payload = (
            ad(0x01, b"\x06")
            + ad(0x09, b"Pixel Buds")
            + ad(0x0A, b"\xf4")
            + ad(0x03, b"\x0f\x18")
            + mfr(0x00E0, b"\x01\x02\x03")
        )
        adv = decode_advertisement(payload, "7F:AA:BB:CC:DD:EE", "random", -58)
        self.assertEqual(adv.name, "Pixel Buds")
        self.assertEqual(adv.manufacturer, "Google")
        self.assertEqual(adv.privacy, "resolvable-random")

    def test_truncated_tail_still_decodes_head(self):
        payload = ad(0x09, b"Govee_H5075") + bytes([30, 0xFF]) + b"\x4c\x00"
        adv = decode_advertisement(payload, "AA:BB:CC:DD:EE:FF", "public", -60)
        self.assertEqual(adv.name, "Govee_H5075")  # good head survived bad tail


if __name__ == "__main__":
    unittest.main(verbosity=2)
