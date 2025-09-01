# Generating a 1024x1024 U-shaped nav mesh with ~100 triangles and some quads,
# plus spawns and a clean 1024x1024 PNG (no axes).
import json
import math
import random
import struct
from pathlib import Path
from typing import TypedDict

import matplotlib.pyplot as plt  # pyright: ignore[reportMissingModuleSource]
from matplotlib.patches import Polygon  # pyright: ignore[reportMissingModuleSource]


class AreaDict(TypedDict):
    id: int
    corners: list[dict[str, float]]
    z_mean: float


random.seed(42)

# Grid setup: 8x8 cells => cell size 125 to cover 1024x1024
cols = 8
rows = 8
cell = 1024 / cols  # 128.0

# Determine which grid cells belong to U-shape
# We'll take:
# - Bottom bar: rows 0..2 (y from 0 to 384) all columns
# - Left arm: cols 0..2, rows 3..7
# - Right arm: cols 5..7, rows 3..7
u_cells: set[tuple[int, int]] = set()
# bottom bar
for r in range(3):
    for c in range(cols):
        u_cells.add((c, r))
# left arm
for r in range(3, rows):
    for c in range(3):
        u_cells.add((c, r))
# right arm
for r in range(3, rows):
    for c in range(5, 8):
        u_cells.add((c, r))

# Ramp region: right arm lower half -> we'll use rows 3..4 as ramp (2 rows)
ramp_rows = (3, 4, 5, 6, 7)
ramp_x_cols = (7,)
ramp_y_min = ramp_rows[0] * cell
ramp_y_max = (ramp_rows[-1] + 1) * cell  # top of ramp region
ramp_z_top = 150.0

areas: list[AreaDict] = []
area_id = 1


def cell_corners(c: int, r: int) -> list[tuple[float, float]]:
    x0 = c * cell
    y0 = r * cell
    x1 = x0 + cell
    y1 = y0 + cell
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


# Build list of cells to be quads (small percentage ~10-15%)
u_cells_list = sorted(u_cells)
num_cells = len(u_cells_list)
num_quads = int(num_cells * 0.12)  # ~12% quads
quad_candidates = random.sample(u_cells_list, num_quads)
quad_set = set(quad_candidates)

for c, r in u_cells_list:
    corners = cell_corners(c, r)

    # compute z for each corner: ramp interpolation if within ramp region
    def z_for_y(y: float, c: int = c) -> float:
        # if within ramp X columns and ramp rows in Y, interpolate
        if (c in ramp_x_cols) and (ramp_y_min <= y <= ramp_y_max):
            t = (y - ramp_y_min) / (ramp_y_max - ramp_y_min)
            return t * ramp_z_top
        return 0.0

    corners3d = [{"x": float(x), "y": float(y), "z": float(z_for_y(y))} for (x, y) in corners]
    if (c, r) in quad_set:
        # keep as quad (4 corners)
        areas.append(
            {
                "id": area_id,
                "corners": [corners3d[0], corners3d[1], corners3d[2], corners3d[3]],
                "z_mean": sum(pt["z"] for pt in corners3d) / 4.0,
            }
        )
        area_id += 1
    else:
        # split into two triangles with alternating diagonal for variety
        if (c + r) % 2 == 0:
            tri1 = [corners3d[0], corners3d[1], corners3d[2]]
            tri2 = [corners3d[0], corners3d[2], corners3d[3]]
        else:
            tri1 = [corners3d[0], corners3d[1], corners3d[3]]
            tri2 = [corners3d[1], corners3d[2], corners3d[3]]
        areas.append({"id": area_id, "corners": tri1, "z_mean": sum(pt["z"] for pt in tri1) / len(tri1)})
        area_id += 1
        areas.append({"id": area_id, "corners": tri2, "z_mean": sum(pt["z"] for pt in tri2) / len(tri2)})
        area_id += 1

# Build nav JSON structure
nav = {"version": 1, "sub_version": 0, "is_analyzed": True, "areas": {}}


def shares_two_points(corners_a: list[dict[str, float]], corners_b: list[dict[str, float]], tol: float = 1e-6) -> bool:
    """Return True if polygons a and b share at least two points (x,y,z)."""
    shared = 0
    for pa in corners_a:
        for pb in corners_b:
            if (
                math.isclose(pa["x"], pb["x"], abs_tol=tol)
                and math.isclose(pa["y"], pb["y"], abs_tol=tol)
                and math.isclose(pa["z"], pb["z"], abs_tol=tol)
            ):
                shared += 1
                if shared >= 2:
                    return True
    return False


for a in areas:
    nav["areas"][str(a["id"])] = {
        "area_id": a["id"],
        "hull_index": 0,
        "dynamic_attribute_flags": 0,
        "corners": [{"x": pt["x"], "y": pt["y"], "z": pt["z"]} for pt in a["corners"]],
        "connections": [
            b["id"]
            for b in areas
            if a["id"] != b["id"] and shares_two_points(a["corners"], b["corners"]) and random.random() < 0.5  # noqa: S311
        ],
        "ladders_above": [],
        "ladders_below": [],
    }

# Spawns: CT in left arm center, T on flat top and T on ramp mid
ct_spawn = {"x": float(cell * 1.5), "y": float(cell * 6.5), "z": 0.0}
t_flat = {"x": float(cell * 6.5), "y": float(cell * 6.5), "z": 0.0}
# T on ramp: pick mid ramp Y
t_ramp_y = (ramp_y_min + ramp_y_max) / 2.0
t_ramp = {
    "x": float(7.5 * cell),
    "y": float(5.5 * cell),
    "z": round(((t_ramp_y - ramp_y_min) / (ramp_y_max - ramp_y_min)) * ramp_z_top, 6),
}

spawns = {"CT": [ct_spawn], "T": [t_flat, t_ramp]}

# Save files
nav_path = Path(__file__) / "../nav/test_good.json"
spawns_path = Path(__file__) / "../spawns/test_good.json"
with open(nav_path, "w") as f:
    json.dump(nav, f, indent=2)
    f.write("\n")
with open(spawns_path, "w") as f:
    json.dump(spawns, f, indent=2)
    f.write("\n")

# Create PNG: 1024x1024 pixels exactly. Use figsize and dpi such that pixels = 1024
dpi = 100
figsize = (10.24, 10.24)  # inches -> 10.24*100 = 1024 px
fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

# Background similar to reference: dark slate
fig.patch.set_facecolor("#0b0b0b")
ax.set_facecolor("#0b0b0b")

# Draw polygons with subtle shading according to mean z (higher = lighter)
for a in areas:
    zs = [pt["z"] for pt in a["corners"]]
    zmean = sum(zs) / len(zs)
    # map zmean 0..150 to color scale
    t = zmean / ramp_z_top if ramp_z_top else 0.0
    # base color bluish-gray, interpolate toward lighter for higher z
    base = (58 / 255, 80 / 255, 107 / 255)
    light = (170 / 255, 180 / 255, 190 / 255)
    color = tuple(base[i] * (1 - t) + light[i] * t for i in range(3))
    poly = Polygon(
        [(pt["x"], pt["y"]) for pt in a["corners"]], closed=True, facecolor=color, edgecolor="#111111", linewidth=0.6
    )
    ax.add_patch(poly)

# Draw spawns as small markers (but no legend)
ax.scatter(
    [ct_spawn["x"]],
    [ct_spawn["y"]],
    s=120,
    marker="o",
    facecolor="#ffd966",
    edgecolor="#2b2b2b",
    linewidth=0.8,
    zorder=5,
)
ax.scatter(
    [t_flat["x"]], [t_flat["y"]], s=120, marker="s", facecolor="#6ec1a6", edgecolor="#2b2b2b", linewidth=0.8, zorder=5
)
ax.scatter(
    [t_ramp["x"]], [t_ramp["y"]], s=120, marker="^", facecolor="#f06666", edgecolor="#2b2b2b", linewidth=0.8, zorder=5
)

# Remove axes and whitespace, set exact limits
ax.set_xlim(0, 1024)
ax.set_ylim(0, 1024)
ax.set_xticks([])
ax.set_yticks([])
plt.subplots_adjust(left=0, right=1, top=1, bottom=0)

png_path = Path(__file__) / "../maps/test_good.png"
plt.savefig(png_path, dpi=dpi, bbox_inches=None, pad_inches=0)
plt.close()


# Define the wall across the U opening at x=512 , spanning y 0-640, z=0-500
wall_bottom_high = (4 * cell, 8 * cell, 0.0)
wall_bottom_low = (4 * cell, 3 * cell, 0.0)
wall_top_high = (4 * cell, 8 * cell, 500.0)
wall_top_low = (4 * cell, 3 * cell, 500.0)

# Two triangles to form the rectangle wall
triangles = [[wall_bottom_high, wall_bottom_low, wall_top_high], [wall_top_high, wall_bottom_low, wall_top_low]]

tri_path = Path(__file__) / "../tri/test_good.tri"

with open(tri_path, "wb") as f:
    for tri in triangles:
        for x, y, z in tri:
            f.write(struct.pack("f", x))
            f.write(struct.pack("f", y))
            f.write(struct.pack("f", z))

triangles_clippings = triangles
tri_path_clippings = Path(__file__) / "../tri/test_good-clippings.tri"

with open(tri_path_clippings, "wb") as f:
    for tri in triangles_clippings:
        for x, y, z in tri:
            f.write(struct.pack("f", x))
            f.write(struct.pack("f", y))
            f.write(struct.pack("f", z))
