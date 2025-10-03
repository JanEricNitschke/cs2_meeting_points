"""Script for reading and plotting the spread data produced by the Rust code."""

import argparse
import collections
import gc
import itertools
import json
from collections.abc import Iterable
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any, Literal, Self, TypedDict

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection, PatchCollection
from tqdm import tqdm

MeetingStyle = Literal["fine", "rough"]

# -----------------------------------------------------
# Duplication of some awpy code to read the rust data.

# Jumpheigt in hammer units with crouch jumping
JUMP_HEIGHT = 66.02


class DynamicAttributeFlags(int):
    """A custom integer class for dynamic attribute flags."""

    def __new__(cls, value: Any) -> "DynamicAttributeFlags":  # noqa: ANN401
        """Creates a new DynamicAttributeFlags instance.

        Args:
            value: The integer value for the flags.

        Returns:
            A new DynamicAttributeFlags instance.
        """
        return super().__new__(cls, value)


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        """Create a Vector3 instance from a dictionary."""
        return cls(data["x"], data["y"], data["z"])


@dataclass
class NavArea:
    corners: list[Vector3]
    area_id: int = 0
    hull_index: int = 0
    dynamic_attribute_flags: DynamicAttributeFlags = DynamicAttributeFlags(0)  # noqa: RUF009
    connections: list[int] = field(default_factory=list)
    ladders_above: list[int] = field(default_factory=list)
    ladders_below: list[int] = field(default_factory=list)

    @cached_property
    def centroid(self) -> Vector3:
        """Calculates the centroid of the polygon defined by the corners.

        Returns:
            A Vector3 representing the centroid (geometric center) of the polygon.
        """
        if not self.corners:
            return Vector3(0, 0, 0)  # Return origin if no corners exist

        x_coords = [corner.x for corner in self.corners]
        y_coords = [corner.y for corner in self.corners]

        centroid_x = sum(x_coords) / len(self.corners)
        centroid_y = sum(y_coords) / len(self.corners)

        # Assume z is averaged as well for completeness
        z_coords = [corner.z for corner in self.corners]
        centroid_z = sum(z_coords) / len(self.corners)

        return Vector3(centroid_x, centroid_y, centroid_z)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        """Load a NavArea from a dictionary."""
        return cls(
            area_id=data["area_id"],
            hull_index=data["hull_index"],
            dynamic_attribute_flags=DynamicAttributeFlags(data["dynamic_attribute_flags"]),
            corners=[Vector3.from_dict(c) for c in data["corners"]],
            connections=data["connections"],
            ladders_above=data["ladders_above"],
            ladders_below=data["ladders_below"],
        )


@dataclass
class Nav:
    version: int
    sub_version: int
    areas: dict[int, NavArea]
    is_analyzed: bool

    @classmethod
    def from_json(cls, path: str | Path) -> "Nav":
        """Reads the navigation mesh data from a JSON file.

        Args:
            path: Path to the JSON file to read from.
        """
        nav_dict = json.loads(Path(path).read_text())
        return cls(
            version=nav_dict["version"],
            sub_version=nav_dict["sub_version"],
            areas={int(area_id): NavArea.from_dict(area_dict) for area_id, area_dict in nav_dict["areas"].items()},
            is_analyzed=nav_dict["is_analyzed"],
        )


@dataclass
class ReducedSpawnDistance:
    area: int
    path: list[int] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(area=data["area"], path=data["path"])


@dataclass
class SpreadResult:
    new_marked_areas_ct: set[int]
    new_marked_areas_t: set[int]

    visibility_connections: list[tuple[ReducedSpawnDistance, ReducedSpawnDistance]]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(
            new_marked_areas_ct=set(data["new_marked_areas_ct"]),
            new_marked_areas_t=set(data["new_marked_areas_t"]),
            visibility_connections=[
                (ReducedSpawnDistance.from_dict(origin), ReducedSpawnDistance.from_dict(target))
                for origin, target in data["visibility_connections"]
            ],
        )

    @classmethod
    def list_from_json(cls, path: str | Path) -> list[Self]:
        with Path(path).open() as f:
            data = json.load(f)
        return [cls.from_dict(entry) for entry in data]


class VerticalSection(TypedDict):
    """Type for a specified vertical section of a map."""

    altitude_min: float
    altitude_max: float


class MapData(TypedDict):
    """Type of the data for a map. `pos_x` is upper left world coordinate."""

    pos_x: int
    pos_y: int
    scale: float
    rotate: int | None
    zoom: float | None
    vertical_sections: dict[str, VerticalSection]
    lower_level_max_units: float


MAP_DATA: dict[str, MapData] = json.loads((Path(__file__).parent / "../maps/map-data.json").read_bytes())

# -----------------------------------------------------
# Duplication of some awpy code to read the rust data.


def find_level(z_value: float, vertical_sections: dict[str, VerticalSection]) -> tuple[int, str]:
    """Finds the level name and index for a given Z value."""
    if not vertical_sections:
        return 0, "default"

    sorted_keys = sorted(vertical_sections, key=lambda k: vertical_sections[k]["altitude_max"], reverse=True)

    for index, key in enumerate(sorted_keys):
        section = vertical_sections[key]
        if section["altitude_min"] <= z_value <= section["altitude_max"]:
            return index, key  # Return both the index and level name

    # If no match is found, return the lowest (last) level
    lowest_key = sorted_keys[-1]
    return len(sorted_keys) - 1, lowest_key


def game_to_pixel(map_name: str, position: Vector3, *, radar_size: int = 1024) -> tuple[float, float, float]:
    """Transforms a `(X, Y, Z)` CS2-coord to pixel coord.

    Modified from awpy to better support multi level maps.

    Args:
        map_name (str): Map to transform coordinates.
        position (tuple): (X,Y,Z) coordinates.

    Returns:
        Tuple[float, float, float]: Transformed coordinates (X,Y,Z).
    """
    current_map_data = MAP_DATA[map_name]
    pos_x = current_map_data["pos_x"]
    pos_y = current_map_data["pos_y"]
    scale = current_map_data["scale"]
    x = (position.x - pos_x) / scale
    y = (pos_y - position.y) / scale
    z = position.z
    map_vertical_sections = current_map_data.get("vertical_sections", {})
    if map_vertical_sections:
        level, _ = find_level(z, map_vertical_sections)
        y += level * radar_size
    return (x, y, z)


def plot_map(map_name: str) -> tuple[plt.Figure, Axes, int]:
    """Modified from awpy to better support multi level maps."""
    fig, ax = plt.subplots()

    maps_dir = Path("maps")
    map_img_path = maps_dir / f"{map_name}.png"

    # Load and display the map
    vertical_sections = MAP_DATA[map_name]["vertical_sections"] if map_name in MAP_DATA else {}
    if vertical_sections:
        map_bgs = []

        # Sorting by altitude_max in descending order
        for section_name in sorted(vertical_sections, key=lambda k: vertical_sections[k]["altitude_max"], reverse=True):
            section_img_path = (
                maps_dir / f"{map_name}_{section_name}.png" if section_name != "default" else map_img_path
            )
            map_bgs.append(mpimg.imread(section_img_path))
        map_bg = np.concatenate(map_bgs)
    else:
        map_bg = mpimg.imread(map_img_path)

    ax.imshow(map_bg, zorder=0, alpha=0.5)
    ax.axis("off")
    # fig.patch.set_facecolor("black")
    plt.tight_layout()
    fig.set_size_inches(19.2, 10.8 * (max(len(vertical_sections), 1)))
    return fig, ax, map_bg.shape[1]


def _plot_tiles(
    map_areas: dict[int, NavArea],
    map_name: str,
    axis: Axes,
    color: str = "yellow",
    facecolor: str = "None",
    zorder: int = 1,
    linewidth: float = 1.0,
    *,
    radar_size: int = 1024,
) -> None:
    axis.add_collection(
        PatchCollection(
            [
                patches.Polygon(
                    [game_to_pixel(map_name, c, radar_size=radar_size)[0:2] for c in area.corners],
                )
                for area in map_areas.values()
            ],
            linewidth=linewidth,
            edgecolor=color,
            facecolor=facecolor,
            zorder=zorder,
        ),
    )


def _plot_points(
    points: list[Vector3],
    map_name: str,
    axis: Axes,
    color: str = "green",
    marker_size: float = 5,
    marker: str = "o",
    *,
    radar_size: int = 1024,
) -> None:
    for point in points:
        x, y, _ = game_to_pixel(map_name, point, radar_size=radar_size)
        axis.plot(x, y, marker=marker, color=color, markersize=marker_size, alpha=1.0, zorder=10)


def same_map_level(area1: NavArea, area2: NavArea, map_name: str) -> bool:
    area1_level, _ = find_level(area1.centroid.z, MAP_DATA[map_name].get("vertical_sections", {}))
    area2_level, _ = find_level(area2.centroid.z, MAP_DATA[map_name].get("vertical_sections", {}))
    return area1_level == area2_level


def _plot_connection(
    area1: NavArea,
    area2: NavArea,
    map_name: str,
    axis: Axes,
    *,
    with_arrows: bool = False,
    color: str = "red",
    lw: float = 0.3,
    radar_size: int = 1024,
) -> None:
    if same_map_level(area1, area2, map_name):
        x1, y1, _ = game_to_pixel(map_name, area1.centroid, radar_size=radar_size)
        x2, y2, _ = game_to_pixel(map_name, area2.centroid, radar_size=radar_size)
        axis.plot([x1, x2], [y1, y2], color=color, lw=lw)

        if with_arrows:
            axis.annotate(
                "",
                xy=(x2, y2),  # Arrow tip
                xytext=(x1, y1),  # Arrow base
                arrowprops={"arrowstyle": "->", "color": color, "lw": lw},
            )
    # Do not draw from one level to the other across the plot.
    # Instead draw one line on each level.
    else:
        area1_at_2_z = Vector3(area1.centroid.x, area1.centroid.y, area2.centroid.z)
        area2_at_1_z = Vector3(area2.centroid.x, area2.centroid.y, area1.centroid.z)

        x1, y1, _ = game_to_pixel(map_name, area1.centroid, radar_size=radar_size)
        x2, y2, _ = game_to_pixel(map_name, area2_at_1_z, radar_size=radar_size)
        axis.plot([x1, x2], [y1, y2], color=color, lw=lw)

        x1, y1, _ = game_to_pixel(map_name, area1_at_2_z, radar_size=radar_size)
        x2, y2, _ = game_to_pixel(map_name, area2.centroid, radar_size=radar_size)
        axis.plot([x1, x2], [y1, y2], color=color, lw=lw)


def _plot_path(
    path: list[NavArea],
    axis: Axes,
    map_name: str,
    color: str = "green",
    lw: float = 0.3,
    linestyle: str = "solid",
    *,
    radar_size: int = 1024,
) -> None:
    lines = [
        [
            game_to_pixel(map_name, first.centroid, radar_size=radar_size)[:2],
            game_to_pixel(map_name, second.centroid, radar_size=radar_size)[:2],
        ]
        for first, second in itertools.pairwise(path)
        # Skip connections that would go from one level to another
        if same_map_level(first, second, map_name)
    ]
    line_collection = LineCollection(lines, colors=color, linewidths=lw, linestyle=linestyle)
    axis.add_collection(line_collection)


def _plot_visibility_connection(
    area1: ReducedSpawnDistance,
    area2: ReducedSpawnDistance,
    map_nav: Nav,
    map_name: str,
    axis: Axes,
    *,
    color: str = "red",
    lw: float = 1.0,
    radar_size: int = 1024,
) -> None:
    _plot_tiles(
        {0: map_nav.areas[area1.area], 1: map_nav.areas[area2.area]},
        map_name,
        axis,
        color=color,
        radar_size=radar_size,
    )
    _plot_connection(
        map_nav.areas[area1.area],
        map_nav.areas[area2.area],
        map_name,
        axis,
        with_arrows=False,
        color=color,
        lw=lw,
        radar_size=radar_size,
    )
    _plot_path(
        [map_nav.areas[path_id] for path_id in area1.path],
        axis,
        map_name,
        color=color,
        linestyle="dashed",
        lw=lw,
        radar_size=radar_size,
    )
    _plot_path(
        [map_nav.areas[path_id] for path_id in area2.path],
        axis,
        map_name,
        color=color,
        linestyle="dashed",
        lw=lw,
        radar_size=radar_size,
    )


def group_nav_areas(nav_areas: Iterable[NavArea], group_size: int) -> list[list[Vector3]]:
    """Groups nav areas into NxN clusters and returns their boundary positions.

    Should use the same granularity as what is used in the rust code to
    visualize (and debug) the spread algorithm with the grouping."""

    # Find min_x and min_y to normalize cell placement
    min_x = min(area.centroid.x for area in nav_areas)
    min_y = min(area.centroid.y for area in nav_areas)

    # Compute tile size based on first area
    first_area = next(iter(nav_areas))
    tile_min_x = min(c.x for c in first_area.corners)
    tile_min_y = min(c.y for c in first_area.corners)
    tile_max_x = max(c.x for c in first_area.corners)
    tile_max_y = max(c.y for c in first_area.corners)

    delta_x = tile_max_x - tile_min_x
    delta_y = tile_max_y - tile_min_y

    # Group areas into grid cells
    block_map: dict[tuple[int, int], list[NavArea]] = collections.defaultdict(list)
    for area in nav_areas:
        cell_x = round((area.centroid.x - min_x) / delta_x)
        cell_y = round((area.centroid.y - min_y) / delta_y)
        block_map[(cell_x // group_size, cell_y // group_size)].append(area)

    # Process groups, ensuring areas in the same Z-range are kept together
    grouped_boundaries: list[list[Vector3]] = []

    for _, areas in sorted(block_map.items()):
        z_groups: list[list[NavArea]] = []

        for area in areas:
            cell_coord = (
                round((area.centroid.x - min_x) / delta_x),
                round((area.centroid.y - min_y) / delta_y),
            )
            found = False

            for group in z_groups:
                if any(
                    round((a.centroid.x - min_x) / delta_x) == cell_coord[0]
                    and round((a.centroid.y - min_y) / delta_y) == cell_coord[1]
                    for a in group
                ):
                    continue  # Skip if another area in this Z-group shares the same (x, y) cell

                if all(abs(a.centroid.z - area.centroid.z) <= JUMP_HEIGHT for a in group):
                    group.append(area)
                    found = True
                    break

            if not found:
                z_groups.append([area])

        # Compute outer bounds for each Z-group
        for group in z_groups:
            min_x = min(c.x for area in group for c in area.corners)
            min_y = min(c.y for area in group for c in area.corners)
            max_x = max(c.x for area in group for c in area.corners)
            max_y = max(c.y for area in group for c in area.corners)
            avg_z = sum(c.z for area in group for c in area.corners) / sum(1 for area in group for c in area.corners)

            # Store the four boundary positions
            grouped_boundaries.append(
                [
                    Vector3(min_x, min_y, avg_z),
                    Vector3(min_x, max_y, avg_z),
                    Vector3(max_x, max_y, avg_z),
                    Vector3(max_x, min_y, avg_z),
                ]
            )

    return grouped_boundaries


def plot_spread_from_input(map_name: str, style: MeetingStyle) -> None:
    """Plot the spread data from the Rust code."""
    print("Loading spread input.", flush=True)
    nav = Nav.from_json(f"results/{args.map_name}.json")
    spread_input = SpreadResult.list_from_json(Path("results") / f"{map_name}_{style}_spreads.json")
    print("Finished loading spread input.", flush=True)
    marked_areas_ct: set[int] = set()
    marked_areas_t: set[int] = set()

    image_dir = Path("spread_images") / map_name
    image_dir.mkdir(exist_ok=True, parents=True)

    gif_dir = Path("spread_gifs") / map_name
    gif_dir.mkdir(exist_ok=True, parents=True)

    # Create the base plot with the radar image and yellow outlines for all areas.
    fig, axis, radar_size = plot_map(map_name)
    fig.set_size_inches(19.2, 21.6)
    _plot_tiles(
        nav.areas,
        map_name=map_name,
        axis=axis,
        color="yellow",
        radar_size=radar_size,
    )

    # complex_maps and n_grouping have to be kept in sync with the rust code.
    complex_maps = [
        "ar_shoots",
        "ar_shoots_night",
        "ar_baggage",
        "ar_pool_day",
        "de_palais",
        "de_rooftop",
        "de_vertigo",
        "de_whistle",
    ]
    n_grouping = 10
    granularity = 100 if map_name in complex_maps else 200

    groupings = group_nav_areas(nav.areas.values(), round(n_grouping * granularity / 200))

    per_image_axis = fig.add_axes(axis.get_position(), sharex=axis, sharey=axis)
    per_image_axis.axis("off")

    image_names: list[str] = []

    # Loop over each spread point and accumulate the reachable areas for each team.
    # Plot the one that were reachable in a previous step in olive and the new ones in green.
    # Plot the ones that were reachable for both teams in purple.
    # Plot the visibility connections in red and highlight the newly visible areas in red.
    # Also plot the paths to the newly visible areas in dashed red lines.
    for idx, spread_point in enumerate(tqdm(spread_input, desc="Plotting spreads")):
        # Draw the groupings with thin black lines.
        # This has to ge in here and on the `per_image_axis` because z-order is not respected
        # across different axes.
        _plot_tiles(
            {idx: NavArea(corners=corners) for idx, corners in enumerate(groupings)},
            map_name=map_name,
            axis=per_image_axis,
            color="black",
            zorder=2,
            linewidth=0.2,
            radar_size=radar_size,
        )
        _plot_tiles(
            {area_id: nav.areas[area_id] for area_id in (marked_areas_ct | marked_areas_t)},
            map_name=map_name,
            axis=per_image_axis,
            color="olive",
            radar_size=radar_size,
        )
        _plot_tiles(
            {
                area_id: nav.areas[area_id]
                for area_id in (spread_point.new_marked_areas_ct | spread_point.new_marked_areas_t)
            },
            map_name=map_name,
            axis=per_image_axis,
            color="green",
            radar_size=radar_size,
        )

        _plot_tiles(
            {
                area_id: nav.areas[area_id]
                for area_id in (marked_areas_t | spread_point.new_marked_areas_t)
                & (marked_areas_ct | spread_point.new_marked_areas_ct)
            },
            map_name=map_name,
            axis=per_image_axis,
            color="purple",
            radar_size=radar_size,
        )
        marked_areas_ct |= spread_point.new_marked_areas_ct
        marked_areas_t |= spread_point.new_marked_areas_t

        for area1, area2 in spread_point.visibility_connections:
            _plot_visibility_connection(
                area1,
                area2,
                nav,
                map_name,
                per_image_axis,
                color="red",
                lw=1.0,
                radar_size=radar_size,
            )

        image_path = image_dir / f"spread_{map_name}_{idx}.png"
        image_names.append(str(image_path))
        plt.savefig(
            image_path,
            bbox_inches="tight",
            dpi=200,
        )

        # Try to free memory as much as possible to avoid OOM errors on GitHub runners.
        per_image_axis.cla()
        per_image_axis.axis("off")

        gc.collect()

    fig.clear()
    plt.close(fig)
    del nav
    del spread_input
    gc.collect()

    gif_path = gif_dir / "spread.gif"

    webpage_dir_path = Path("webpage_data")
    webpage_dir_path.mkdir(exist_ok=True, parents=True)
    webpage_data_path = webpage_dir_path / f"{map_name}.json"
    webpage_data_path.write_text(json.dumps({map_name: {"gif": str(gif_path), "images": image_names}}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process a map name.")
    parser.add_argument("map_name", type=str, help="Name of the map to process")
    args = parser.parse_args()

    style = "fine"

    plot_spread_from_input(args.map_name, style)
