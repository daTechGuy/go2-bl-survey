"""BLE advertisement decoder — pure, no radio, no ROS, no clock.

A BLE advertisement payload is a flat list of AD structures (Bluetooth Core
spec, Vol 3 Part C §11):

    [length][AD type][ ...length-1 data bytes... ][length][AD type][...]...

`length` counts the type byte plus its data. A zero length marks the end /
padding. This module walks that structure and turns it into an
`Advertisement`, deriving a coarse `category` and an address-`privacy` class.

Everything here is a pure function of its inputs so it can be unit-tested
against captured fixtures — including malformed packets, which must never take
a good packet down with them. This is the BLE analogue of the `iw`-scan parser
in go2-rf-survey: the parsing is the part most likely to be subtly wrong, so it
is isolated and tested first.

The raw bytes come from a passive scanner (BlueZ `btmon`, `bleak`, or an
nRF52840 sniffer) — see the README. Address and address-type are reported by
the controller in the HCI advertising-report event, *not* in the payload, so
they are passed in alongside it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

# --- AD structure type codes we understand (Core spec / Assigned Numbers) ----

AD_FLAGS = 0x01
AD_UUID16_INCOMPLETE = 0x02
AD_UUID16_COMPLETE = 0x03
AD_NAME_SHORT = 0x08
AD_NAME_COMPLETE = 0x09
AD_TX_POWER = 0x0A
AD_SERVICE_DATA_16 = 0x16
AD_MANUFACTURER = 0xFF

# --- Seed lookup tables ------------------------------------------------------
#
# These are deliberately SMALL and only hold entries verified against the
# Bluetooth SIG Assigned Numbers. They are meant to be extended from a real
# capture of your own environment — an unlabelled device returning
# category="unknown" is honest, not a bug. Do not pad these with guessed IDs;
# a wrong label on a heatmap is worse than "unknown".
#
# Company IDs: https://www.bluetooth.com/specifications/assigned-numbers/

COMPANY_IDS = {
    0x004C: "Apple",
    0x0006: "Microsoft",
    0x00E0: "Google",
    0x0075: "Samsung Electronics",
    0x0087: "Garmin",
}

# 16-bit service UUIDs worth recognising on sight.
SERVICE_UUIDS = {
    0x180D: "Heart Rate",
    0x180F: "Battery Service",
    0xFD6F: "Exposure Notification",
    0xFEAA: "Eddystone",
    0xFEED: "Tile",
}

# Apple's manufacturer data is itself a sequence of [type][len][data] TLVs.
# https://arxiv.org/abs/1904.10600 documents the observed types; the ones that
# change the category are called out here.
APPLE_IBEACON = 0x02
APPLE_AIRPRINT = 0x03
APPLE_AIRDROP = 0x05
APPLE_PROXIMITY_PAIRING = 0x07  # AirPods and kin
APPLE_HANDOFF = 0x0C
APPLE_NEARBY_INFO = 0x10  # a regular, unlocked iPhone/Mac nearby
APPLE_FIND_MY = 0x12  # offline-finding beacon — a *separated* AirTag etc.


@dataclass
class Advertisement:
    """One decoded advertisement from one device at one instant."""

    addr: str
    addr_type: str  # 'public' | 'random' — from the HCI report
    rssi: float

    name: Optional[str] = None
    tx_power: Optional[int] = None
    company_id: Optional[int] = None
    manufacturer: Optional[str] = None
    service_uuids: list = field(default_factory=list)  # 16-bit ints
    adv_type: str = "plain"  # ibeacon | eddystone | findmy | mfr | plain
    category: str = "unknown"  # beacon | tracker | wearable | phone | unknown
    privacy: str = ""  # see address_privacy()
    detail: dict = field(default_factory=dict)

    @property
    def persistent(self) -> bool:
        """True if this address does not rotate — i.e. the device is
        trackable over time. The whole privacy point of BLE address
        randomisation is to make this False."""
        return self.privacy in ("public", "static-random")


# --- The AD-structure walker (the part most likely to be quietly wrong) ------


def parse_ad_structures(payload: bytes):
    """Yield (ad_type, data) for each well-formed AD structure in `payload`.

    Stops cleanly at the first zero-length byte (end/padding) or the first
    structure whose declared length runs off the end of the buffer. Everything
    decoded *before* a malformed tail is returned — a truncated packet still
    yields its good leading structures.
    """
    out = []
    i = 0
    n = len(payload)
    while i < n:
        length = payload[i]
        if length == 0:
            break  # end-of-data marker / zero padding
        if i + 1 + length > n:
            break  # declared length overruns the buffer: truncated/garbage
        ad_type = payload[i + 1]
        data = payload[i + 2 : i + 1 + length]  # length-1 data bytes
        out.append((ad_type, data))
        i += 1 + length
    return out


def _signed_byte(b: int) -> int:
    return b - 256 if b >= 128 else b


def _uuid16_list(data: bytes):
    """Little-endian 16-bit UUIDs, packed two bytes each. A trailing odd byte
    (malformed) is ignored rather than raising."""
    return [int.from_bytes(data[j : j + 2], "little") for j in range(0, len(data) - 1, 2)]


# --- Vendor-specific sub-decoders --------------------------------------------


def parse_apple_tlvs(data: bytes):
    """Apple manufacturer payload -> list of (type, data). The payload is the
    bytes *after* the 2-byte company id. Malformed tail stops the walk."""
    out = []
    i = 0
    n = len(data)
    while i + 2 <= n:
        t = data[i]
        length = data[i + 1]
        if i + 2 + length > n:
            break
        out.append((t, data[i + 2 : i + 2 + length]))
        i += 2 + length
    return out


def parse_ibeacon(mfr_data: bytes):
    """Apple payload (after company id) -> iBeacon dict, or None if it is not
    an iBeacon frame. iBeacon = Apple TLV type 0x02, length 0x15 (21 bytes):
    16-byte proximity UUID, 2-byte major, 2-byte minor, 1-byte measured power."""
    if len(mfr_data) < 23 or mfr_data[0] != APPLE_IBEACON or mfr_data[1] != 0x15:
        return None
    body = mfr_data[2:23]
    uuid = body[0:16].hex()
    return {
        "uuid": f"{uuid[0:8]}-{uuid[8:12]}-{uuid[12:16]}-{uuid[16:20]}-{uuid[20:32]}",
        "major": int.from_bytes(body[16:18], "big"),
        "minor": int.from_bytes(body[18:20], "big"),
        "measured_power": _signed_byte(body[20]),
    }


_EDDYSTONE_URL_SCHEMES = {0x00: "http://www.", 0x01: "https://www.", 0x02: "http://", 0x03: "https://"}
_EDDYSTONE_URL_EXPANSIONS = {
    0x00: ".com/", 0x01: ".org/", 0x02: ".edu/", 0x03: ".net/", 0x04: ".info/",
    0x05: ".biz/", 0x06: ".gov/", 0x07: ".com", 0x08: ".org", 0x09: ".edu",
    0x0A: ".net", 0x0B: ".info", 0x0C: ".biz", 0x0D: ".gov",
}


def parse_eddystone(service_data: bytes):
    """Eddystone service-data payload (after the 0xFEAA UUID) -> dict.
    Recognises UID (0x00), URL (0x10) and TLM (0x20) frame types."""
    if not service_data:
        return None
    frame = service_data[0]
    if frame == 0x00 and len(service_data) >= 18:  # UID
        return {
            "frame": "uid",
            "namespace": service_data[2:12].hex(),
            "instance": service_data[12:18].hex(),
        }
    if frame == 0x10 and len(service_data) >= 3:  # URL
        scheme = _EDDYSTONE_URL_SCHEMES.get(service_data[2], "")
        url = scheme + "".join(
            _EDDYSTONE_URL_EXPANSIONS.get(b, chr(b)) for b in service_data[3:]
        )
        return {"frame": "url", "tx_power": _signed_byte(service_data[1]), "url": url}
    if frame == 0x20 and len(service_data) >= 14:  # TLM
        return {
            "frame": "tlm",
            "battery_mv": int.from_bytes(service_data[2:4], "big"),
        }
    return {"frame": f"0x{frame:02x}"}


# --- Address privacy ---------------------------------------------------------


def address_privacy(addr: str, addr_type: str) -> str:
    """Classify how trackable an advertised address is.

    Public addresses are a device's permanent, globally-unique identity.
    Random addresses encode their sub-type in the two most-significant bits of
    the most-significant octet (Core spec Vol 6 Part B §1.3):

        0b11  static-random      fixed until reboot — effectively trackable
        0b01  resolvable         rotates (~15 min); defeats passive tracking
        0b00  non-resolvable     rotates; no identity at all
        0b10  reserved

    The rotating cases are the privacy feature working. Showing an audience
    that their phone is 'resolvable-random' while a cheap sensor is 'public' is
    the whole lesson.
    """
    if addr_type == "public":
        return "public"
    if addr_type != "random":
        return "unknown"  # controller didn't tell us; don't guess trackability
    try:
        msb = int(addr.split(":")[0], 16)
    except (ValueError, IndexError):
        return "unknown"
    top2 = msb >> 6
    return {
        0b11: "static-random",
        0b01: "resolvable-random",
        0b00: "non-resolvable-random",
        0b10: "reserved-random",
    }[top2]


# --- Category -----------------------------------------------------------------


def classify(adv: Advertisement) -> str:
    """Coarse device category from the fields already decoded. Conservative on
    purpose: returns 'unknown' unless there is a defensible signal. Extend the
    seed tables rather than widening these rules with guesses."""
    if adv.adv_type == "ibeacon" or adv.adv_type == "eddystone":
        return "beacon"
    if adv.adv_type == "findmy":
        return "tracker"
    if adv.company_id == 0x004C and adv.detail.get("apple_type") == APPLE_PROXIMITY_PAIRING:
        return "wearable"
    if 0xFEED in adv.service_uuids:  # Tile
        return "tracker"
    if 0x180D in adv.service_uuids:  # Heart Rate
        return "wearable"
    if adv.company_id == 0x0087:  # Garmin
        return "wearable"
    if adv.company_id in (0x004C, 0x0006, 0x00E0, 0x0075):  # Apple/MS/Google/Samsung
        return "phone"  # continuity / nearby chatter from phones & computers
    return "unknown"


# --- Top-level decode --------------------------------------------------------


def decode_advertisement(payload: bytes, addr: str, addr_type: str, rssi: float) -> Advertisement:
    """Decode one raw advertisement payload into an `Advertisement`.

    Never raises on malformed input: whatever can be decoded is, and the rest
    is left at its default. This is the single entry point the scan node calls
    per advertising report.
    """
    adv = Advertisement(addr=addr, addr_type=addr_type, rssi=float(rssi))
    adv.privacy = address_privacy(addr, addr_type)

    for ad_type, data in parse_ad_structures(payload):
        if ad_type in (AD_NAME_COMPLETE, AD_NAME_SHORT):
            adv.name = data.decode("utf-8", "replace")
        elif ad_type == AD_TX_POWER and data:
            adv.tx_power = _signed_byte(data[0])
        elif ad_type in (AD_UUID16_COMPLETE, AD_UUID16_INCOMPLETE):
            adv.service_uuids.extend(_uuid16_list(data))
        elif ad_type == AD_SERVICE_DATA_16 and len(data) >= 2:
            uuid = int.from_bytes(data[0:2], "little")
            if uuid not in adv.service_uuids:
                adv.service_uuids.append(uuid)
            if uuid == 0xFEAA:  # Eddystone
                adv.adv_type = "eddystone"
                adv.detail["eddystone"] = parse_eddystone(data[2:])
        elif ad_type == AD_MANUFACTURER and len(data) >= 2:
            adv.company_id = int.from_bytes(data[0:2], "little")
            adv.manufacturer = COMPANY_IDS.get(adv.company_id)
            body = data[2:]
            if adv.adv_type == "plain":
                adv.adv_type = "mfr"
            if adv.company_id == 0x004C:
                _decode_apple(adv, body)

    adv.category = classify(adv)
    return adv


def advertisement_from_parsed(
    addr: str,
    addr_type: str,
    rssi: float,
    *,
    name: Optional[str] = None,
    tx_power: Optional[int] = None,
    manufacturer_data: Optional[dict] = None,
    service_uuids: Optional[list] = None,
    service_data: Optional[dict] = None,
) -> Advertisement:
    """Build an `Advertisement` from already-parsed fields.

    Some backends (notably BlueZ via `bleak`) hand you the advertisement
    already split into manufacturer_data `{company_id: bytes}`, 16-bit
    `service_uuids`, and service_data `{uuid16: bytes}` — the leading `length`/
    `type` framing is gone. This maps those onto the *same* vendor sub-decoders
    and classifier `decode_advertisement()` uses, so the two capture paths
    (parsed backend vs. raw sniffer bytes) can never disagree on a category.
    """
    adv = Advertisement(addr=addr, addr_type=addr_type, rssi=float(rssi))
    adv.privacy = address_privacy(addr, addr_type)
    adv.name = name
    adv.tx_power = tx_power
    adv.service_uuids = list(service_uuids or [])

    for uuid, data in (service_data or {}).items():
        if uuid not in adv.service_uuids:
            adv.service_uuids.append(uuid)
        if uuid == 0xFEAA:  # Eddystone
            adv.adv_type = "eddystone"
            adv.detail["eddystone"] = parse_eddystone(data)

    for company_id, body in (manufacturer_data or {}).items():
        adv.company_id = company_id
        adv.manufacturer = COMPANY_IDS.get(company_id)
        if adv.adv_type == "plain":
            adv.adv_type = "mfr"
        if company_id == 0x004C:
            _decode_apple(adv, body)
        break  # more than one manufacturer block in one advert is pathological

    adv.category = classify(adv)
    return adv


def _decode_apple(adv: Advertisement, body: bytes) -> None:
    """Fill adv_type/detail from an Apple manufacturer payload (after company
    id). Sets the first meaningful type seen; iBeacon and Find My win."""
    ibeacon = parse_ibeacon(body)
    if ibeacon is not None:
        adv.adv_type = "ibeacon"
        adv.detail["ibeacon"] = ibeacon
        adv.detail["apple_type"] = APPLE_IBEACON
        return
    for t, _ in parse_apple_tlvs(body):
        adv.detail.setdefault("apple_type", t)
        if t == APPLE_FIND_MY:
            adv.adv_type = "findmy"
            adv.detail["apple_type"] = APPLE_FIND_MY
            return
