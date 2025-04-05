from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import final

@final
@dataclass
class Position:
    x: float
    y: float
    z: float

    def __sub__(self, other: Position) -> Position: ...
    def __add__(self, other: Position) -> Position: ...
    def __mul__(self, other: float) -> Position: ...
    def __truediv__(self, other: float) -> Position: ...
    def dot(self, other: Position) -> float: ...
    def cross(self, other: Position) -> Position: ...
    def length(self) -> float: ...
    def normalize(self) -> Position: ...
    def distance(self, other: Position) -> float: ...
    def distance_2d(self, other: Position) -> float: ...
    def can_jump_to(self, other: Position) -> bool: ...
    def __iter__(self) -> Iterator[float]: ...
    @staticmethod
    def from_input(value: Position | list[float] | tuple[float, float, float] | Sequence[float]) -> Position: ...

def inverse_distance_weighting(points: list[Position], target: tuple[float, float]) -> float: ...
@final
class DynamicAttributeFlags(int):
    def __init__(self, value: int) -> None: ...

@final
@dataclass(frozen=True)
class NavArea:
    area_id: int  # Note, no default
    hull_index: int
    dynamic_attribute_flags: DynamicAttributeFlags
    corners: list[Position]
    connections: list[int]
    ladders_above: list[int]
    ladders_below: list[int]

    @property
    def size(self) -> float: ...
    @property
    def centroid(self) -> Position: ...
    def contains(self, point: Position) -> bool: ...
    def centroid_distance(self, point: Position) -> float: ...

@final
@dataclass
class PathResult:
    path: list[NavArea]
    distance: float

@final
@dataclass(frozen=True)
class Nav:
    version: int
    sub_version: int
    areas: dict[int, NavArea]
    is_analyzed: bool

    def find_area(self, position: Position) -> NavArea | None: ...
    def find_closest_area_centroid(self, position: Position) -> NavArea: ...
    def find_path(self, start: int | Position, end: int | Position) -> PathResult: ...
    def to_json(self, path: Path | str) -> None: ...
    @staticmethod
    def from_json(path: Path | str) -> Nav: ...
    @staticmethod
    def from_path(path: Path | str) -> Nav: ...

def regularize_nav_areas(
    nav_areas: dict[int, NavArea], grid_granularity: int, walk_checker: VisibilityChecker
) -> dict[int, NavArea]: ...
def group_nav_areas(nav_areas: list[NavArea], group_size: int) -> dict[int, int]: ...
@final
@dataclass
class Triangle:
    p1: Position
    p2: Position
    p3: Position

    def get_centroid(self) -> Position: ...
    def ray_intersection(self, ray_origin: Position, ray_direction: Position) -> float | None: ...

@final
class VisibilityChecker:
    def __init__(self, path: Path | str | None = None, triangles: list[Triangle] | None = None) -> None: ...
    @property
    def n_triangles(self) -> int: ...
    def is_visible(
        self,
        start: Position | list[float] | tuple[float, float, float] | Sequence[float],
        end: Position | list[float] | tuple[float, float, float] | Sequence[float],
    ) -> bool: ...
    @staticmethod
    def read_tri_file(tri_file: Path | str, buffer_size: int = 1000) -> list[Triangle]: ...

class InvalidNavFileError(Exception): ...

__all__ = [
    "DynamicAttributeFlags",
    "InvalidNavFileError",
    "Nav",
    "NavArea",
    "PathResult",
    "Position",
    "Triangle",
    "VisibilityChecker",
    "group_nav_areas",
    "inverse_distance_weighting",
    "regularize_nav_areas",
]
