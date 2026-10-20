"""Join a sighting to the robot pose it was taken at — the ROS-free half.

On the robot a sighting is timestamped at scan completion and must be paired
with where the robot was *then*. That pairing is pure bookkeeping over a buffer
of (stamp, pose) samples, so it lives here with no ROS and no clock and is
pinned by tests — exactly the seam go2-rf-survey keeps testable. The phase-03
ROS node is a thin shell: feed TF poses in via `add`, call `lookup` per
sighting.

Two rules carried over from go2-rf-survey's D4 and R4:

* a sighting whose nearest pose is staler than `max_age_s` is **discarded**,
  not placed at a guessed location — a misplaced sample is worse than a missing
  one;
* when two poses bracket the sighting time, interpolate (yaw on the shortest
  arc) rather than snap, so a sighting taken between two TF updates lands
  between them.
"""

from __future__ import annotations

import bisect
import math
from typing import Optional


def _lerp_angle(a: float, b: float, t: float) -> float:
    """Interpolate an angle along the shortest arc (handles the pi wrap)."""
    d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + d * t


class PoseBuffer:
    """A time-ordered buffer of robot poses, bounded in size.

    `max_samples` caps memory on a long survey; the oldest samples fall off.
    Adds need not be in order — the buffer keeps itself sorted by stamp.
    """

    def __init__(self, max_samples: int = 4000):
        self._stamps: list[float] = []
        self._poses: list[dict] = []
        self._max = max_samples

    def __len__(self) -> int:
        return len(self._stamps)

    def add(self, stamp: float, x: float, y: float, z: float, yaw: float) -> None:
        i = bisect.bisect_left(self._stamps, stamp)
        self._stamps.insert(i, stamp)
        self._poses.insert(i, {"x": x, "y": y, "z": z, "yaw": yaw})
        if len(self._stamps) > self._max:
            self._stamps.pop(0)
            self._poses.pop(0)

    def lookup(self, stamp: float, max_age_s: float) -> Optional[dict]:
        """Pose at `stamp`, or None if the buffer has no fresh pose near it.

        Interpolates between the two bracketing poses only when the query is
        surrounded by a pose within `max_age_s` on *each* side — i.e. TF was
        actually streaming across the sighting. Otherwise it snaps to the
        single nearest pose if that is within `max_age_s`, and otherwise
        discards (returns None): a sighting across a TF gap is placed nowhere
        rather than at a guessed location.
        """
        if not self._stamps:
            return None

        i = bisect.bisect_left(self._stamps, stamp)

        # bracketed and fresh on both sides -> interpolate
        if 0 < i < len(self._stamps):
            lo, hi = self._stamps[i - 1], self._stamps[i]
            if (stamp - lo) <= max_age_s and (hi - stamp) <= max_age_s:
                t = 0.0 if hi == lo else (stamp - lo) / (hi - lo)
                a, b = self._poses[i - 1], self._poses[i]
                return {
                    "x": a["x"] + (b["x"] - a["x"]) * t,
                    "y": a["y"] + (b["y"] - a["y"]) * t,
                    "z": a["z"] + (b["z"] - a["z"]) * t,
                    "yaw": _lerp_angle(a["yaw"], b["yaw"], t),
                }

        # fall back to the single nearest sample
        cands = []
        if i < len(self._stamps):
            cands.append(i)
        if i > 0:
            cands.append(i - 1)
        j = min(cands, key=lambda k: abs(self._stamps[k] - stamp))
        if abs(self._stamps[j] - stamp) > max_age_s:
            return None
        return dict(self._poses[j])
