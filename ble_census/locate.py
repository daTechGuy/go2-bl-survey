"""Estimate where a device is, from the geotagged sightings of it.

The classroom payoff: a persistent device (a beacon, a smart-home sensor, a
tracker that isn't rotating its address) is heard from many robot positions,
each with an RSSI. The robot was closest where the signal was strongest, so an
RSSI-weighted centroid of the sighting positions lands on the device. This is
"find the hidden tracker", and because it is pure it can be checked against
known ground truth on synthetic data.

Caveats this function cannot fix and the caller must respect:

* It only makes sense for a **persistent** address. A phone rotating its BLE
  address shows up as many short-lived addresses; averaging across them is
  meaningless. `locate_rows` filters on the `privacy` column for this reason.
* The estimate is a centroid, so it is pulled toward wherever the robot spent
  sightings. It localizes a device *to a region*, not to centimetres.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Iterable, Optional


def locate(sightings: Iterable[tuple]) -> Optional[dict]:
    """RSSI-weighted position estimate from (x, y, rssi) sightings.

    Weight is relative linear power (10**(rssi/10)), so a -50 dBm sighting
    counts ~100x a -70 dBm one and the estimate is dominated by the nearest
    approaches. Returns None for no sightings. Returns `{x, y, n, best_rssi}`.
    """
    sx = sy = wsum = 0.0
    n = 0
    best = None
    for x, y, rssi in sightings:
        w = 10.0 ** (rssi / 10.0)
        sx += w * x
        sy += w * y
        wsum += w
        n += 1
        best = rssi if best is None else max(best, rssi)
    if n == 0 or wsum == 0.0:
        return None
    return {"x": sx / wsum, "y": sy / wsum, "n": n, "best_rssi": best}


def locate_rows(rows: Iterable[dict], min_sightings: int = 4, persistent_only: bool = True) -> dict:
    """Group census rows by `addr` and locate each qualifying device.

    `rows` are dicts with at least `addr`, `x`, `y`, `rssi`, and (if
    `persistent_only`) `privacy`. Rows with a blank pose are skipped — a
    sighting with no position can't vote. Returns `{addr: estimate}`.
    """
    persistent = {"public", "static-random"}
    grouped: dict[str, list[tuple]] = defaultdict(list)
    privacy: dict[str, str] = {}
    for r in rows:
        if persistent_only and r.get("privacy") not in persistent:
            continue
        try:
            x = float(r["x"])
            y = float(r["y"])
            rssi = float(r["rssi"])
        except (ValueError, TypeError, KeyError):
            continue  # blank pose (off-robot row) or malformed — can't place it
        grouped[r["addr"]].append((x, y, rssi))
        privacy.setdefault(r["addr"], r.get("privacy", ""))

    out = {}
    for addr, pts in grouped.items():
        if len(pts) < min_sightings:
            continue
        est = locate(pts)
        if est is not None:
            est["privacy"] = privacy.get(addr, "")
            out[addr] = est
    return out
