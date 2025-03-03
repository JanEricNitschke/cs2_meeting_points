import argparse
import io
import itertools
import json
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any, Literal, Self, TypedDict

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from matplotlib.axes import Axes
from matplotlib.collections import PatchCollection
from PIL import Image
from tqdm import tqdm

MeetingStyle = Literal["fine", "rough"]


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
    area_id: int
    hull_index: int
    dynamic_attribute_flags: DynamicAttributeFlags
    corners: list[Vector3]
    connections: list[int]
    ladders_above: list[int]
    ladders_below: list[int]

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
class SpawnDistance:
    area: NavArea
    distance: float
    path: list[int] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(
            area=NavArea.from_dict(data["area"]),
            distance=data["distance"],
            path=data["path"],
        )


@dataclass
class SpreadResult:
    new_marked_areas_ct: set[int]
    new_marked_areas_t: set[int]

    visibility_connections: list[tuple[SpawnDistance, SpawnDistance]]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(
            new_marked_areas_ct=set(data["new_marked_areas_ct"]),
            new_marked_areas_t=set(data["new_marked_areas_t"]),
            visibility_connections=[
                (SpawnDistance.from_dict(origin), SpawnDistance.from_dict(target))
                for origin, target in data["visibility_connections"]
            ],
        )

    @classmethod
    def list_from_json(cls, path: str | Path) -> list[Self]:
        return [cls.from_dict(entry) for entry in json.loads(Path(path).read_text())]


class MapData(TypedDict):
    """Type of the data for a map. `pos_x` is upper left world coordinate."""

    pos_x: int
    pos_y: int
    scale: float
    rotate: int | None
    zoom: float | None
    lower_level_max_units: float


MAP_DATA: dict[str, MapData] = json.loads((Path(__file__).parent / "../maps/map_data.json").read_bytes())


def game_to_pixel(map_name: str, position: Vector3) -> tuple[float, float, float]:
    """Transforms a `(X, Y, Z)` CS2-coord to pixel coord.

    Args:
        map_name (str): Map to transform coordinates.
        position (tuple): (X,Y,Z) coordinates.

    Returns:
        Tuple[float, float, float]: Transformed coordinates (X,Y,Z).
    """
    current_map_data = MAP_DATA[map_name]
    start_x = current_map_data["pos_x"]
    start_y = current_map_data["pos_y"]
    scale = current_map_data["scale"]
    x = position.x - start_x
    x /= scale
    y = start_y - position.y
    y /= scale
    z = position.z
    if "z_cutoff" in current_map_data and z < current_map_data["z_cutoff"]:
        y += 1024
    return (x, y, z)


def plot_map(map_name: str) -> tuple[plt.Figure, Axes]:
    fig, ax = plt.subplots()

    maps_dir = Path("maps")
    map_img_path = maps_dir / f"{map_name}.png"
    lower_img_path = maps_dir / f"{map_name}_lower.png"

    # Load and display the map
    map_bg = mpimg.imread(map_img_path)

    if map_name in MAP_DATA and "z_cutoff" in MAP_DATA[map_name]:
        map_bg_lower = mpimg.imread(lower_img_path)
        map_bg = np.concatenate([map_bg, map_bg_lower])

    ax.imshow(map_bg, zorder=0, alpha=0.5)
    ax.axis("off")
    # fig.patch.set_facecolor("black")
    plt.tight_layout()
    fig.set_size_inches(19.2, 21.6)
    return fig, ax


def _plot_tiles(
    map_areas: dict[int, NavArea],
    map_name: str,
    axis: Axes,
    color: str = "yellow",
    facecolor: str = "None",
    zorder: int = 1,
    *,
    show_z: bool = False,
) -> None:
    if show_z:
        for area in map_areas.values():
            x, y, _ = game_to_pixel(map_name, area.centroid)
            axis.text(
                x,
                y,
                str(round(area.centroid.z)),
                fontsize=2,
                color="black",
                ha="center",
            )
    axis.add_collection(
        PatchCollection(
            [
                patches.Polygon(
                    [game_to_pixel(map_name, c)[0:2] for c in area.corners],
                )
                for area in map_areas.values()
            ],
            linewidth=1,
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
) -> None:
    for point in points:
        x, y, _ = game_to_pixel(map_name, point)
        axis.plot(
            x,
            y,
            marker=marker,
            color=color,
            markersize=marker_size,
            alpha=1.0,
            zorder=10,
        )


def _plot_connection(
    area1: NavArea,
    area2: NavArea,
    map_name: str,
    axis: Axes,
    *,
    with_arrows: bool = False,
    color: str = "red",
    lw: float = 0.3,
) -> None:
    x1, y1, _ = game_to_pixel(map_name, area1.centroid)
    x2, y2, _ = game_to_pixel(map_name, area2.centroid)
    axis.plot([x1, x2], [y1, y2], color=color, lw=lw)

    if with_arrows:
        axis.annotate(
            "",
            xy=(x2, y2),  # Arrow tip
            xytext=(x1, y1),  # Arrow base
            arrowprops={"arrowstyle": "->", "color": color, "lw": lw},
        )


def _plot_path(
    path: list[NavArea],
    axis: Axes,
    map_name: str,
    color: str = "green",
    lw: float = 0.3,
    linestyle: str = "solid",
) -> None:
    for first, second in itertools.pairwise(path):
        x1, y1, _ = game_to_pixel(map_name, first.centroid)
        x2, y2, _ = game_to_pixel(map_name, second.centroid)
        axis.plot([x1, x2], [y1, y2], color=color, lw=lw, linestyle=linestyle)


def _plot_visibility_connection(
    area1: SpawnDistance,
    area2: SpawnDistance,
    map_nav: Nav,
    map_name: str,
    axis: Axes,
    *,
    color: str = "red",
    lw: float = 1.0,
    highlight_area1: bool = False,
) -> None:
    if highlight_area1:
        _plot_points([area1.area.centroid], map_name, axis, color="yellow")
    _plot_tiles({0: area1.area, 1: area2.area}, map_name, axis, color=color)
    _plot_connection(area1.area, area2.area, map_name, axis, with_arrows=False, color=color, lw=lw)
    _plot_path(
        [map_nav.areas[path_id] for path_id in area1.path],
        axis,
        map_name,
        color=color,
        linestyle="dashed",
        lw=lw,
    )
    _plot_path(
        [map_nav.areas[path_id] for path_id in area2.path],
        axis,
        map_name,
        color=color,
        linestyle="dashed",
        lw=lw,
    )


def plot_spread_from_input(map_name: str, granularity: str, style: MeetingStyle, nav: Nav) -> None:
    print("Loading spread input.", flush=True)
    spread_input = SpreadResult.list_from_json(Path("results") / f"{map_name}_{style}_spreads_{granularity}.json")
    print("Finished loading spread input.", flush=True)
    marked_areas_ct: set[int] = set()
    marked_areas_t: set[int] = set()

    image_dir = Path("spread_images") / map_name
    image_dir.mkdir(exist_ok=True, parents=True)

    gif_dir = Path("spread_gifs") / map_name
    gif_dir.mkdir(exist_ok=True, parents=True)

    frames = []

    for idx, spread_point in enumerate(tqdm(spread_input[:30], desc="Plotting spreads.")):
        contains_new_connection = bool(spread_point.visibility_connections)
        new_conn_str = "_new" if contains_new_connection else ""

        fig, axis = plot_map(map_name)
        fig.set_size_inches(19.2, 21.6)

        _plot_tiles(
            {area_id: nav.areas[area_id] for area_id in (marked_areas_ct | marked_areas_t)},
            map_name=map_name,
            axis=axis,
            color="olive",
        )
        _plot_tiles(
            {
                area_id: nav.areas[area_id]
                for area_id in (spread_point.new_marked_areas_ct | spread_point.new_marked_areas_t)
            },
            map_name=map_name,
            axis=axis,
            color="green",
        )

        _plot_tiles(
            {
                area_id: nav.areas[area_id]
                for area_id in (marked_areas_t | spread_point.new_marked_areas_t)
                & (marked_areas_ct | spread_point.new_marked_areas_ct)
            },
            map_name=map_name,
            axis=axis,
            color="purple",
        )
        marked_areas_ct |= spread_point.new_marked_areas_ct
        marked_areas_t |= spread_point.new_marked_areas_t
        _plot_tiles(
            {
                area_id: area
                for area_id, area in nav.areas.items()
                if area_id not in marked_areas_ct and area_id not in marked_areas_t
            },
            map_name=map_name,
            axis=axis,
            color="yellow",
        )

        for area1, area2 in spread_point.visibility_connections:
            _plot_visibility_connection(
                area1,
                area2,
                nav,
                map_name,
                axis,
                color="red",
                lw=1.0,
                highlight_area1=style == "rough",
            )

        plt.savefig(
            image_dir / f"spread_{map_name}_{granularity}{new_conn_str}_{idx}.png",
            bbox_inches="tight",
            dpi=300,
        )


        buf = io.BytesIO()
        buf.seek(0)
        fig.savefig(buf, format="png") # , facecolor="black"
        img = Image.open(buf)
        frames.append(img)

        fig.clear()
        plt.close(fig)

    frames[0].save(
        gif_dir / "spread.gif",
        save_all=True,
        append_images=frames[1:],
        duration=[400 if bool(entry.visibility_connections) else 200 for entry in spread_input],
        disposal=2,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process a map name.")
    parser.add_argument("map_name", type=str, help="Name of the map to process")
    args = parser.parse_args()

    granularity = 200
    style = "fine"

    plot_spread_from_input(args.map_name, granularity, style, Nav.from_json(f"nav/{args.map_name}_{granularity}.json"))
