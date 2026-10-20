"""Read and write ROS ``map_server`` occupancy grids (PGM + YAML).

Kept dependency-light on purpose: numpy only, no ROS. The heatmap tools run
off-robot, often on a laptop with no ROS installed at all.

The map_server convention:

* ``<name>.pgm`` -- greyscale image, row 0 is the **top** of the map
* ``<name>.yaml`` -- ``resolution`` (m/px), ``origin`` [x, y, yaw] of the
  **bottom-left** corner in world coordinates, plus occupancy thresholds

The vertical flip between image rows and world Y is the single most common
source of mirrored heatmaps, so it is handled in exactly one place here.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FREE, OCCUPIED, UNKNOWN = 0, 1, 2


@dataclass
class OccupancyGrid:
    """Occupancy grid in world coordinates.

    ``cells`` is indexed [row, col] with row 0 at **minimum** Y (i.e. already
    flipped from image order), so that increasing row means increasing world Y.
    Values are FREE / OCCUPIED / UNKNOWN.
    """

    cells: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def height(self) -> int:
        return self.cells.shape[0]

    @property
    def width(self) -> int:
        return self.cells.shape[1]

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """(xmin, xmax, ymin, ymax) in metres, for matplotlib ``imshow``."""
        return (
            self.origin_x,
            self.origin_x + self.width * self.resolution,
            self.origin_y,
            self.origin_y + self.height * self.resolution,
        )

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def is_free(self, x: float, y: float) -> bool:
        row, col = self.world_to_cell(x, y)
        if not (0 <= row < self.height and 0 <= col < self.width):
            return False
        return bool(self.cells[row, col] == FREE)


def _read_pgm(path: Path) -> np.ndarray:
    """Minimal PGM reader for P2 (ascii) and P5 (binary)."""
    raw = path.read_bytes()

    # Strip comments, then pull the header tokens.
    def tokens(buf: bytes, count: int) -> tuple[list[bytes], int]:
        out: list[bytes] = []
        i = 0
        while len(out) < count:
            while i < len(buf) and buf[i : i + 1].isspace():
                i += 1
            if buf[i : i + 1] == b"#":
                while i < len(buf) and buf[i : i + 1] != b"\n":
                    i += 1
                continue
            start = i
            while i < len(buf) and not buf[i : i + 1].isspace():
                i += 1
            out.append(buf[start:i])
        return out, i

    (magic, w, h, maxval), idx = tokens(raw, 4)
    width, height = int(w), int(h)

    if magic == b"P5":
        idx += 1  # single whitespace byte after maxval
        dtype = np.uint8 if int(maxval) < 256 else ">u2"
        data = np.frombuffer(raw[idx:], dtype=dtype, count=width * height)
    elif magic == b"P2":
        nums = re.findall(rb"\d+", raw[idx:])
        data = np.array([int(n) for n in nums[: width * height]], dtype=np.uint16)
    else:
        raise ValueError(f"{path}: unsupported PGM magic {magic!r} (want P2 or P5)")

    return data.reshape(height, width)


def load(map_yaml: str | Path) -> OccupancyGrid:
    """Load a map_server map. Accepts the .yaml path."""
    yaml_path = Path(map_yaml)
    text = yaml_path.read_text()

    def field(name: str, default=None):
        m = re.search(rf"^\s*{name}\s*:\s*(.+?)\s*$", text, re.MULTILINE)
        if m is None:
            if default is None:
                raise ValueError(f"{yaml_path}: missing required field {name!r}")
            return default
        return m.group(1)

    image = field("image").strip().strip("\"'")
    resolution = float(field("resolution"))
    origin_raw = field("origin")
    ox, oy = (float(v) for v in re.findall(r"-?\d+\.?\d*", origin_raw)[:2])
    occupied_thresh = float(field("occupied_thresh", "0.65"))
    free_thresh = float(field("free_thresh", "0.196"))
    negate = int(float(field("negate", "0")))

    img = _read_pgm(yaml_path.parent / image).astype(np.float64)
    maxv = 255.0

    # map_server: p = (255 - value) / 255, unless negated.
    p = img / maxv if negate else (maxv - img) / maxv

    cells = np.full(p.shape, UNKNOWN, dtype=np.uint8)
    cells[p > occupied_thresh] = OCCUPIED
    cells[p < free_thresh] = FREE

    # PGM row 0 is the top of the map; flip so row 0 is minimum world Y.
    cells = np.flipud(cells)

    return OccupancyGrid(cells, resolution, ox, oy)


def save(grid: OccupancyGrid, map_yaml: str | Path) -> None:
    """Write a map_server-compatible PGM + YAML pair."""
    yaml_path = Path(map_yaml)
    pgm_path = yaml_path.with_suffix(".pgm")

    img = np.full(grid.cells.shape, 205, dtype=np.uint8)   # unknown
    img[grid.cells == FREE] = 254
    img[grid.cells == OCCUPIED] = 0
    img = np.flipud(img)                                    # back to image order

    h, w = img.shape
    with pgm_path.open("wb") as fh:
        fh.write(b"P5\n")
        fh.write(b"# generated by go2-rf-survey tools/occupancy.py\n")
        fh.write(f"{w} {h}\n255\n".encode())
        fh.write(img.tobytes())

    yaml_path.write_text(
        f"image: {pgm_path.name}\n"
        f"resolution: {grid.resolution}\n"
        f"origin: [{grid.origin_x}, {grid.origin_y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
