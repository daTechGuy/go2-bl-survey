#!/usr/bin/env python3
"""Generate a synthetic floorplan and BLE census — ground truth included.

Makes the post-processing chain verifiable *before* the robot exists, exactly
as go2-rf-survey's synthetic survey does. It writes the same artefacts a real
run produces — a map_server map and a census CSV in the schema writer.py emits
— plus a truth CSV of where the devices actually are, so heatmap.py and
locate.py can be checked: we know where the hidden tracker is, so we can see
whether localization finds it.

    python tools/make_synthetic_census.py --out docs/sample

Writes <out>.pgm, <out>.yaml, <out>.csv and <out>-truth.csv.

The propagation model is log-distance with wall attenuation, tuned for a BLE
advertiser (~0 dBm, 2.4 GHz) rather than an AP. Good enough to exercise
rendering, masking and localization; not a substitute for the real building.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

import occupancy
from occupancy import FREE, OCCUPIED, OccupancyGrid

# --- BLE advertiser propagation ---------------------------------------------
# RSSI(d) = RSSI_1M - 10*n*log10(d) - walls*WAF + jitter
RSSI_1M_DBM = -59.0      # a typical beacon's calibrated measured power at 1 m
PATH_LOSS_N = 2.6        # indoor exponent
WALL_ATTEN_DB = 5.0      # per interior wall crossed (2.4 GHz)
HEARD_FLOOR_DBM = -92.0  # below this the robot does not log the advertisement
RSSI_JITTER_DB = 2.5     # measurement noise, 1 sigma


def build_floorplan(res: float = 0.05) -> OccupancyGrid:
    """A 30 x 18 m floor: central corridor, rooms either side (shared with the
    go2-rf-survey fixture so the two projects render over the same geometry)."""
    w_m, h_m = 30.0, 18.0
    cols, rows = int(w_m / res), int(h_m / res)
    cells = np.full((rows, cols), OCCUPIED, dtype=np.uint8)

    def carve(x0, y0, x1, y1):
        cells[int(y0 / res):int(y1 / res), int(x0 / res):int(x1 / res)] = FREE

    carve(1.0, 8.0, 29.0, 10.5)                       # corridor
    for x0 in (1.5, 8.0, 14.5, 21.0):                 # north rooms
        carve(x0, 11.0, x0 + 5.5, 16.5)
        carve(x0 + 2.3, 10.5, x0 + 3.2, 11.0)
    for x0 in (1.5, 9.5, 17.5):                       # south rooms
        carve(x0, 2.0, x0 + 7.0, 7.5)
        carve(x0 + 3.0, 7.5, x0 + 3.9, 8.0)
    carve(25.0, 3.0, 29.0, 7.5)                       # south-east space
    carve(26.0, 7.5, 28.0, 8.0)
    return OccupancyGrid(cells, res, 0.0, 0.0)


def count_walls(grid, ax, ay, bx, by) -> int:
    dist = math.hypot(bx - ax, by - ay)
    steps = max(2, int(dist / (grid.resolution * 0.5)))
    crossings, was_occ = 0, False
    for i in range(steps + 1):
        t = i / steps
        row, col = grid.world_to_cell(ax + (bx - ax) * t, ay + (by - ay) * t)
        now_occ = not (0 <= row < grid.height and 0 <= col < grid.width) or \
            grid.cells[row, col] == OCCUPIED
        crossings += now_occ and not was_occ
        was_occ = now_occ
    return crossings


def rssi_at(grid, dev, x, y, rng) -> float | None:
    d = max(0.5, math.hypot(x - dev["x"], y - dev["y"]))
    walls = count_walls(grid, dev["x"], dev["y"], x, y)
    rssi = (
        RSSI_1M_DBM
        - 10.0 * PATH_LOSS_N * math.log10(d)
        - walls * WALL_ATTEN_DB
        + rng.normal(0.0, RSSI_JITTER_DB)
    )
    return None if rssi < HEARD_FLOOR_DBM else rssi


def plan_route(grid, spacing: float = 1.5):
    pts = []

    def run(x0, y0, x1, y1):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / spacing))
        for i in range(n + 1):
            t = i / n
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            if grid.is_free(x, y):
                pts.append((x, y))

    run(2.0, 9.2, 28.0, 9.2)
    for x0 in (1.5, 8.0, 14.5, 21.0):
        cx = x0 + 2.75
        run(cx, 10.0, cx, 15.5)
        run(x0 + 0.8, 13.0, x0 + 4.7, 13.0)
    for x0 in (1.5, 9.5):  # third south room left unvisited on purpose
        cx = x0 + 3.45
        run(cx, 8.5, cx, 3.0)
        run(x0 + 0.8, 4.5, x0 + 6.2, 4.5)
    run(27.0, 8.0, 27.0, 4.0)
    return pts


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="docs/sample", help="output path prefix")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--spacing", type=float, default=1.5)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    grid = build_floorplan()

    # Ground truth. A mix of categories and privacy classes, including one
    # persistent tracker tucked in a room (the "find the hidden AirTag" target)
    # and a rotating phone that localization must refuse to place.
    devices = [
        {"addr": "c3:00:00:00:be:01", "category": "beacon", "privacy": "public",
         "adv_type": "eddystone", "company_id": "", "name": "", "x": 5.0, "y": 13.0},
        {"addr": "c3:00:00:00:5e:02", "category": "smarthome", "privacy": "public",
         "adv_type": "mfr", "company_id": "0x004c", "name": "Govee_H5075", "x": 11.0, "y": 14.0},
        {"addr": "c3:00:00:00:a1:03", "category": "tracker", "privacy": "static-random",
         "adv_type": "findmy", "company_id": "0x004c", "name": "", "x": 24.0, "y": 5.5},
        {"addr": "c3:00:00:00:7e:04", "category": "wearable", "privacy": "public",
         "adv_type": "mfr", "company_id": "0x0087", "name": "", "x": 3.5, "y": 4.5},
        {"addr": "7f:00:00:00:db:05", "category": "phone", "privacy": "resolvable-random",
         "adv_type": "mfr", "company_id": "0x004c", "name": "", "x": 17.0, "y": 9.5},
    ]

    occupancy.save(grid, out.with_suffix(".yaml"))
    with Path(f"{out}-truth.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["addr", "category", "privacy", "x", "y"])
        w.writeheader()
        for d in devices:
            w.writerow({k: d[k] for k in ("addr", "category", "privacy", "x", "y")})

    route = plan_route(grid, args.spacing)
    fieldnames = [
        "stamp", "sample_id", "x", "y", "z", "yaw",
        "addr", "addr_type", "privacy", "rssi", "tx_power",
        "name", "company_id", "adv_type", "category",
    ]
    rows = 0
    with out.with_suffix(".csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames)
        w.writeheader()
        t = 1760932400.0
        for sid, (x, y) in enumerate(route):
            t += 4.0 + args.spacing / 0.35
            yaw = rng.uniform(-math.pi, math.pi)
            for d in devices:
                r = rssi_at(grid, d, x, y, rng)
                if r is None:
                    continue  # genuinely not heard -- absence is data
                addr_type = "public" if d["privacy"] == "public" else "random"
                w.writerow({
                    "stamp": f"{t:.3f}", "sample_id": sid,
                    "x": f"{x:.4f}", "y": f"{y:.4f}", "z": "0.4000", "yaw": f"{yaw:.5f}",
                    "addr": d["addr"], "addr_type": addr_type, "privacy": d["privacy"],
                    "rssi": f"{r:.1f}", "tx_power": "", "name": d["name"],
                    "company_id": d["company_id"], "adv_type": d["adv_type"],
                    "category": d["category"],
                })
                rows += 1

    print(f"floorplan  {out.with_suffix('.pgm')}  ({grid.width}x{grid.height} @ {grid.resolution} m)")
    print(f"census     {out.with_suffix('.csv')}  ({len(route)} samples, {rows} rows)")
    print(f"truth      {out.name}-truth.csv  ({len(devices)} devices)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
