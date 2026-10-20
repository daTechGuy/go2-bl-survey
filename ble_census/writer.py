"""CSV schema for the census — one row per device-sighting per position.

Pure and dependency-free: `to_row()` is a plain dict transform so it can be
tested without a file or a radio, and the on-robot node (phase 03) only has to
supply the pose. Column order matches the schema table in the README.
"""

from __future__ import annotations

import csv
from typing import Optional

from .ble_scan import Advertisement

FIELDNAMES = [
    "stamp",
    "sample_id",
    "x",
    "y",
    "z",
    "yaw",
    "addr",
    "addr_type",
    "privacy",
    "rssi",
    "tx_power",
    "name",
    "company_id",
    "adv_type",
    "category",
]


def to_row(adv: Advertisement, pose: Optional[dict] = None) -> dict:
    """One CSV row for one sighting. `pose` carries stamp/sample_id/x/y/z/yaw
    when the sighting is geotagged on the robot; off-robot (phases 01-02) it is
    None and those columns are left blank — never zero, which would plot at the
    map origin and lie."""
    pose = pose or {}
    return {
        "stamp": pose.get("stamp", ""),
        "sample_id": pose.get("sample_id", ""),
        "x": pose.get("x", ""),
        "y": pose.get("y", ""),
        "z": pose.get("z", ""),
        "yaw": pose.get("yaw", ""),
        "addr": adv.addr,
        "addr_type": adv.addr_type,
        "privacy": adv.privacy,
        "rssi": adv.rssi,
        "tx_power": "" if adv.tx_power is None else adv.tx_power,
        "name": adv.name or "",
        "company_id": "" if adv.company_id is None else f"0x{adv.company_id:04x}",
        "adv_type": adv.adv_type,
        "category": adv.category,
    }


class CensusWriter:
    """Streams rows to a CSV, flushing each one so a dead battery costs the
    last sighting, not the run — the same guarantee go2-rf-survey's writer makes."""

    def __init__(self, fileobj):
        self._w = csv.DictWriter(fileobj, fieldnames=FIELDNAMES)
        self._w.writeheader()
        self._file = fileobj

    def write(self, adv: Advertisement, pose: Optional[dict] = None) -> None:
        self._w.writerow(to_row(adv, pose))
        self._file.flush()
