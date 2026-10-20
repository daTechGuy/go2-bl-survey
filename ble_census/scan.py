"""Live BLE capture for a workstation — `python -m ble_census.scan`.

This is the phase-01 bring-up and the phase-02 table-extension tool: point a
real adapter at a room, watch devices resolve into categories, and dump a CSV
to drive heatmaps or to find the vendor IDs worth adding to `ble_scan.py`.

It uses `bleak` (cross-platform: BlueZ, Windows, macOS). `bleak` hands us the
advertisement already parsed, so each report goes through
`advertisement_from_parsed()` — the *same* classifier the raw-sniffer path
uses. On the robot, capture switches to a passive nRF52840 sniffer feeding
`decode_advertisement()` directly; this CLI is the off-robot cousin.

    pip install -r requirements.txt
    python -m ble_census.scan --seconds 20
    python -m ble_census.scan --seconds 60 --csv room.csv

Note on scan mode: a dev workstation uses an active scan to see devices
quickly. The *robot* census is passive-only by design (README) — that is a
property of the deployment sniffer, not this bring-up tool.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from typing import Optional

from .ble_scan import Advertisement, advertisement_from_parsed
from .census import Aggregator
from .writer import CensusWriter

_BASE_UUID_SUFFIX = "-0000-1000-8000-00805f9b34fb"


def uuid16(uuid: str) -> Optional[int]:
    """128-bit UUID string -> its 16-bit short form, or None if it is a true
    128-bit (vendor) UUID with no 16-bit equivalent."""
    u = uuid.lower()
    if u.endswith(_BASE_UUID_SUFFIX) and u.startswith("0000"):
        return int(u[4:8], 16)
    return None


def _addr_type(device) -> str:
    """Best-effort public/random from the backend. BlueZ exposes it; Windows
    and macOS usually don't, so privacy falls back to 'unknown' rather than a
    guess."""
    details = getattr(device, "details", None)
    try:
        if isinstance(details, dict):
            props = details.get("props", details)
            at = props.get("AddressType")
            if at in ("public", "random"):
                return at
    except Exception:
        pass
    return ""


def advertisement_from_bleak(device, adv_data) -> Advertisement:
    """Map a bleak (device, AdvertisementData) pair onto an Advertisement via
    the shared parsed-fields path."""
    service_uuids = [s for s in (uuid16(u) for u in adv_data.service_uuids) if s is not None]
    service_data = {}
    for u, data in adv_data.service_data.items():
        s = uuid16(u)
        if s is not None:
            service_data[s] = data
    return advertisement_from_parsed(
        device.address,
        _addr_type(device),
        adv_data.rssi,
        name=adv_data.local_name,
        tx_power=adv_data.tx_power,
        manufacturer_data=dict(adv_data.manufacturer_data),
        service_uuids=service_uuids,
        service_data=service_data,
    )


async def run_scan(seconds: float, csv_path: Optional[str]) -> Aggregator:
    try:
        from bleak import BleakScanner
    except ImportError:
        sys.exit("bleak is not installed — run: pip install -r requirements.txt")

    agg = Aggregator()
    writer = None
    csv_file = None
    if csv_path:
        csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        writer = CensusWriter(csv_file)

    def on_detect(device, adv_data):
        adv = advertisement_from_bleak(device, adv_data)
        agg.add(adv)
        if writer is not None:
            writer.write(adv)  # no pose off-robot — those columns stay blank

    scanner = BleakScanner(detection_callback=on_detect)
    print(f"scanning {seconds:.0f}s ...  (Ctrl-C to stop early)")
    await scanner.start()
    try:
        await asyncio.sleep(seconds)
    finally:
        await scanner.stop()
        if csv_file is not None:
            csv_file.close()
    return agg


def print_report(agg: Aggregator) -> None:
    summaries = agg.summaries()
    print(f"\n{len(summaries)} distinct devices\n")
    print(f"{'rssi':>5}  {'privacy':<21} {'category':<9} {'manufacturer':<20} name")
    print("-" * 78)
    for d in summaries:
        print(
            f"{d.best_rssi:>5.0f}  {d.privacy:<21} {d.category:<9} "
            f"{d.manufacturer:<20} {d.name}"
        )

    counts = agg.category_counts()
    print("\nby category:", ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))
    frac = agg.persistent_fraction()
    print(
        f"persistent (trackable) addresses: {frac:.0%} "
        f"-- the rest rotate, which is the privacy feature working"
    )


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Live BLE census for a workstation.")
    p.add_argument("--seconds", type=float, default=20.0, help="scan duration (default 20)")
    p.add_argument("--csv", default=None, help="write one row per sighting to this file")
    args = p.parse_args(argv)

    try:
        agg = asyncio.run(run_scan(args.seconds, args.csv))
    except KeyboardInterrupt:
        return 130
    print_report(agg)
    if args.csv:
        print(f"\nrows written to {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
