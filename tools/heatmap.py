#!/usr/bin/env python3
"""Render a BLE census CSV as per-category coverage heatmaps over the floorplan.

    python tools/heatmap.py docs/sample.csv --map docs/sample.yaml --out docs/
    python tools/heatmap.py docs/sample.csv --map docs/sample.yaml --truth docs/sample-truth.csv --out docs/

Runs off-robot. numpy + scipy + matplotlib, no ROS.

One panel per device category (beacon, tracker, smart-home, wearable, phone),
coloured by the strongest RSSI of any device in that category heard at each
position — "where would you hear a tracker at all". Persistent devices are
localized (ble_census.locate) and their estimated positions drawn as markers;
with --truth, the real positions are drawn too, with an error line, so the
whole chain is checkable against known ground truth.

Two masks, both load-bearing (same as go2-rf-survey):

* **occupancy** — walls and unmapped cells are never coloured.
* **sample distance** — a cell further than --max-reach from a real sample is
  left blank, so a room the robot never entered stays blank instead of being
  invented by the interpolator.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, Normalize  # noqa: E402
from scipy.interpolate import griddata  # noqa: E402
from scipy.ndimage import distance_transform_edt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # repo root
import occupancy  # noqa: E402
from occupancy import FREE  # noqa: E402
from ble_census.locate import locate_rows  # noqa: E402

RSSI_MIN, RSSI_MAX = -95.0, -45.0
SIGNAL_CMAP = LinearSegmentedColormap.from_list(
    "signal",
    ["#5B1A12", "#A93B2B", "#D98324", "#E8C547", "#5FAE8C", "#0C7C72", "#06403C"],
)
GRID_RES_M = 0.10
CATEGORY_ORDER = ["tracker", "beacon", "smarthome", "wearable", "phone", "unknown"]


def load_census(path: Path) -> list[dict]:
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"{path}: no rows")
    required = {"x", "y", "rssi", "addr", "category"}
    missing = required - set(rows[0])
    if missing:
        raise SystemExit(f"{path}: missing columns {sorted(missing)}")
    out = []
    for r in rows:
        try:
            out.append({
                "x": float(r["x"]), "y": float(r["y"]), "rssi": float(r["rssi"]),
                "addr": r["addr"], "category": r["category"] or "unknown",
                "privacy": r.get("privacy", ""),
            })
        except (ValueError, TypeError):
            continue  # blank pose (off-robot row) or malformed — skip, don't abort
    return out


def best_per_position(rows: list[dict]) -> list[dict]:
    """Strongest RSSI per (position) — the best any device in this group offers
    there, which is what a coverage question actually asks."""
    best: dict[tuple, dict] = {}
    for r in rows:
        key = (round(r["x"], 2), round(r["y"], 2))
        if key not in best or r["rssi"] > best[key]["rssi"]:
            best[key] = r
    return list(best.values())


def interpolate(rows, grid, max_reach_m, bounds):
    xmin, xmax, ymin, ymax = bounds
    nx = max(2, int((xmax - xmin) / GRID_RES_M))
    ny = max(2, int((ymax - ymin) / GRID_RES_M))
    gx, gy = np.linspace(xmin, xmax, nx), np.linspace(ymin, ymax, ny)
    mesh_x, mesh_y = np.meshgrid(gx, gy)
    pts = np.array([[r["x"], r["y"]] for r in rows])
    vals = np.array([r["rssi"] for r in rows])

    if len(pts) < 4:
        field = griddata(pts, vals, (mesh_x, mesh_y), method="nearest")
    else:
        field = griddata(pts, vals, (mesh_x, mesh_y), method="linear")
        near = griddata(pts, vals, (mesh_x, mesh_y), method="nearest")
        field = np.where(np.isnan(field), near, field)
    masked = np.ma.masked_invalid(field)

    sampled = np.zeros((ny, nx), dtype=bool)
    for r in rows:
        col = int((r["x"] - xmin) / (xmax - xmin) * (nx - 1))
        row = int((r["y"] - ymin) / (ymax - ymin) * (ny - 1))
        if 0 <= row < ny and 0 <= col < nx:
            sampled[row, col] = True
    cell_m = max((xmax - xmin) / nx, (ymax - ymin) / ny)
    dist = distance_transform_edt(~sampled)
    masked = np.ma.masked_where(dist > max_reach_m / cell_m, masked)

    if grid is not None:
        cols = np.floor((gx - grid.origin_x) / grid.resolution).astype(int)
        grows = np.floor((gy - grid.origin_y) / grid.resolution).astype(int)
        in_x = (cols >= 0) & (cols < grid.width)
        in_y = (grows >= 0) & (grows < grid.height)
        cc, rr = np.meshgrid(np.clip(cols, 0, grid.width - 1), np.clip(grows, 0, grid.height - 1))
        free = (grid.cells[rr, cc] == FREE) & np.outer(in_y, in_x)
        masked = np.ma.masked_where(~free, masked)
    return masked, (xmin, xmax, ymin, ymax)


def draw_panel(ax, field, extent, grid, rows, title, markers):
    if grid is not None:
        walls = np.ma.masked_where(grid.cells == FREE, grid.cells)
        ax.imshow(walls, extent=grid.extent, origin="lower", cmap="Greys",
                  vmin=0, vmax=3, alpha=0.5, interpolation="nearest")
    im = ax.imshow(field, extent=extent, origin="lower", cmap=SIGNAL_CMAP,
                   norm=Normalize(RSSI_MIN, RSSI_MAX), interpolation="bilinear", zorder=2)
    ax.plot([r["x"] for r in rows], [r["y"] for r in rows], ".",
            color="#11191C", markersize=1.4, alpha=0.4, zorder=3)
    for m in markers:
        if m["kind"] == "est":
            ax.plot(m["x"], m["y"], "o", mfc="none", mec="#0B132B", mew=1.8,
                    markersize=13, zorder=5, label="estimated")
        else:  # truth
            ax.plot(m["x"], m["y"], "*", color="#0B132B", markersize=13, zorder=6, label="actual")
        if "to" in m:
            ax.plot([m["x"], m["to"][0]], [m["y"], m["to"][1]], "-",
                    color="#0B132B", lw=1.0, alpha=0.8, zorder=4)
    ax.set_title(title, fontsize=11, fontweight="bold", loc="left")
    ax.set_xlabel("x (m)", fontsize=8)
    ax.set_ylabel("y (m)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_aspect("equal")
    return im


def load_truth(path: Path | None) -> dict:
    if path is None:
        return {}
    with path.open(newline="") as fh:
        return {r["addr"]: {"x": float(r["x"]), "y": float(r["y"]),
                            "category": r["category"]} for r in csv.DictReader(fh)}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--map", type=Path, default=None)
    ap.add_argument("--truth", type=Path, default=None, help="truth CSV for localization check")
    ap.add_argument("--out", type=Path, default=Path("."))
    ap.add_argument("--max-reach", type=float, default=3.0, metavar="M")
    ap.add_argument("--min-samples", type=int, default=8)
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    rows = load_census(args.csv)
    grid = occupancy.load(args.map) if args.map else None
    truth = load_truth(args.truth)
    args.out.mkdir(parents=True, exist_ok=True)

    if grid is not None:
        bounds = grid.extent
    else:
        xs, ys = [r["x"] for r in rows], [r["y"] for r in rows]
        bounds = (min(xs) - 2, max(xs) + 2, min(ys) - 2, max(ys) + 2)

    located = locate_rows(rows, min_sightings=args.min_samples)
    located_cat = {a: next((r["category"] for r in rows if r["addr"] == a), "unknown")
                   for a in located}

    by_cat: dict[str, list[dict]] = defaultdict(list)
    for r in rows:
        by_cat[r["category"]].append(r)
    cats = [c for c in CATEGORY_ORDER if c in by_cat] + \
           [c for c in by_cat if c not in CATEGORY_ORDER]
    panels = [c for c in cats if len(by_cat[c]) >= args.min_samples]
    if not panels:
        raise SystemExit("no category has enough samples to render")

    print(f"{len(rows)} sightings, {len(by_cat)} categories, {len(located)} device(s) located")
    for addr, est in located.items():
        line = f"  located {addr} ({located_cat[addr]}) at ({est['x']:.1f}, {est['y']:.1f}) n={est['n']}"
        if addr in truth:
            err = math.hypot(est["x"] - truth[addr]["x"], est["y"] - truth[addr]["y"])
            line += f"  -- truth ({truth[addr]['x']:.1f}, {truth[addr]['y']:.1f}), error {err:.2f} m"
        print(line)

    ncols = min(2, len(panels))
    nrows = math.ceil(len(panels) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(7.4 * ncols, 4.9 * nrows), squeeze=False)
    fig.patch.set_facecolor("#FAFBFB")

    im = None
    for idx, cat in enumerate(panels):
        ax = axes[idx // ncols][idx % ncols]
        crows = best_per_position(by_cat[cat])
        field, extent = interpolate(crows, grid, args.max_reach, bounds)
        markers = []
        for addr, est in located.items():
            if located_cat[addr] != cat:
                continue
            m = {"kind": "est", "x": est["x"], "y": est["y"]}
            if addr in truth:
                m["to"] = (truth[addr]["x"], truth[addr]["y"])
                markers.append({"kind": "truth", "x": truth[addr]["x"], "y": truth[addr]["y"]})
            markers.append(m)
        n_dev = len({r["addr"] for r in by_cat[cat]})
        im = draw_panel(ax, field, extent, grid, crows,
                        f"{cat}  ({n_dev} device(s), {len(by_cat[cat])} sightings)", markers)

    for idx in range(len(panels), nrows * ncols):
        axes[idx // ncols][idx % ncols].axis("off")

    cbar = fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.55, pad=0.02, aspect=28)
    cbar.set_label("strongest RSSI (dBm)", fontsize=9)
    cbar.ax.tick_params(labelsize=8)
    for lo, hi, label, ink in [
        (-55, -45, "immediate", "#FFFFFF"), (-70, -55, "near", "#11191C"),
        (-80, -70, "mid", "#11191C"), (-95, -80, "far", "#FFFFFF"),
    ]:
        cbar.ax.axhline(hi, color="#11191C", linewidth=0.7, alpha=0.35)
        cbar.ax.text(0.5, (lo + hi) / 2.0, label, fontsize=6.5, va="center", ha="center",
                     rotation=90, transform=cbar.ax.get_yaxis_transform(), color=ink, fontweight="bold")

    fig.suptitle(f"BLE census — {args.csv.name}", fontsize=13, fontweight="bold", x=0.065, ha="left")
    note = (f"Colour = strongest RSSI of any device in the category. "
            f"Blank = no sample within {args.max_reach:.1f} m. Grey = wall/unmapped. "
            f"○ estimated device location, ★ actual.")
    fig.text(0.065, 0.015, note, fontsize=7.5, color="#65757A")

    out_png = args.out / f"{args.csv.stem}-census.png"
    fig.savefig(out_png, dpi=args.dpi, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"wrote {out_png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
