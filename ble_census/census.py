"""In-memory aggregation of sightings for the live view.

Pure: the scan CLI feeds `Advertisement`s in and reads a sorted summary out, so
the "what's in the room" logic is testable without a radio. On the robot this
is per-waypoint; at a workstation it's the whole session.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from .ble_scan import Advertisement


@dataclass
class DeviceSummary:
    addr: str
    privacy: str
    category: str
    best_rssi: float  # closest the robot ever got (strongest signal)
    count: int  # how many advertisements seen
    name: str = ""
    manufacturer: str = ""


class Aggregator:
    """Collapses many sightings of one address into a single summary, keeping
    the strongest RSSI (nearest approach) and the richest name/category seen."""

    def __init__(self):
        self._by_addr: Dict[str, DeviceSummary] = {}

    def add(self, adv: Advertisement) -> None:
        cur = self._by_addr.get(adv.addr)
        if cur is None:
            self._by_addr[adv.addr] = DeviceSummary(
                addr=adv.addr,
                privacy=adv.privacy,
                category=adv.category,
                best_rssi=adv.rssi,
                count=1,
                name=adv.name or "",
                manufacturer=adv.manufacturer or "",
            )
            return
        cur.count += 1
        cur.best_rssi = max(cur.best_rssi, adv.rssi)
        # later packets often carry detail the first one lacked
        if adv.category != "unknown":
            cur.category = adv.category
        if adv.name and not cur.name:
            cur.name = adv.name
        if adv.manufacturer and not cur.manufacturer:
            cur.manufacturer = adv.manufacturer

    def __len__(self) -> int:
        return len(self._by_addr)

    def summaries(self) -> List[DeviceSummary]:
        """Strongest signal first — the devices nearest the robot lead."""
        return sorted(self._by_addr.values(), key=lambda d: d.best_rssi, reverse=True)

    def category_counts(self) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for d in self._by_addr.values():
            counts[d.category] = counts.get(d.category, 0) + 1
        return counts

    def persistent_fraction(self) -> float:
        """Share of distinct devices whose address does not rotate — the
        headline privacy number. 0.0 when nothing has been seen."""
        if not self._by_addr:
            return 0.0
        persistent = sum(1 for d in self._by_addr.values() if d.privacy in ("public", "static-random"))
        return persistent / len(self._by_addr)
