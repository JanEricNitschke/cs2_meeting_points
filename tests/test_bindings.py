import tempfile
from pathlib import Path

import pytest
from cs2_nav import (
    DynamicAttributeFlags,
    InvalidNavFileError,
    Nav,
    NavArea,
    PathResult,
    Position,
    Triangle,
    VisibilityChecker,
    group_nav_areas,
    inverse_distance_weighting,
    regularize_nav_areas,
)

DATA_PATH = Path(__file__).parent / "data"


def test_position() -> None:
    pos1 = Position(1, 2, 3)
    pos2 = Position(4, 5, 6)
    a, b, c = pos1
    assert (a, b, c) == (1, 2, 3)
    assert (pos1 + pos2).x == 5
    assert (pos1 - pos2).y == -3
    assert (pos1 * 2) == Position(2, 4, 6)
    assert (pos1 / 2) == Position(0.5, 1, 1.5)
    assert isinstance(pos1.dot(pos2), float)
    assert isinstance(pos1.cross(pos2), Position)
    assert isinstance(pos1.length(), float)
    assert isinstance(pos1.normalize(), Position)
    assert isinstance(pos1.distance(pos2), float)
    assert isinstance(pos1.distance_2d(pos2), float)
    assert isinstance(pos1.can_jump_to(pos2), bool)

    pos3 = Position.from_input((1, 2, 3))
    pos4 = Position.from_input([1, 2, 3])
    pos5 = Position.from_input(Position(1, 2, 3))
    assert pos3 == pos4 == pos5

    with pytest.raises(ValueError, match="Input must be a Vector3 or tuple or list of length 3"):
        Position.from_input([1, 2])


def test_inverse_distance_weighting() -> None:
    pos1 = Position(1, 2, 3)
    pos2 = Position(4, 5, 6)
    assert isinstance(inverse_distance_weighting([pos1, pos2], (2, 4)), float)


def test_dynamic_attribute_flags() -> None:
    flag1 = DynamicAttributeFlags(1)
    flag2 = DynamicAttributeFlags(1)
    assert flag1 == flag2


def test_nav_area() -> None:
    area = NavArea(
        1,
        2,
        DynamicAttributeFlags(1),
        [Position(0, 0, 0), Position(1, 0, 0), Position(1, 1, 0), Position(0, 1, 0)],
        [1, 2],
        [1, 2],
        [1, 2],
    )
    assert isinstance(area.size, float)
    assert isinstance(area.centroid, Position)
    assert area.contains(Position(0.5, 0.5, 0))
    assert isinstance(area.centroid_distance(Position(0.5, 0.5, 0)), float)


def test_nav() -> None:
    pos1 = Position(0.5, 0.5, 0)
    pos_1bad = Position(0.5, -1, 0)
    pos2 = Position(0.5, 2.5, 0)
    area1 = NavArea(
        1,
        0,
        DynamicAttributeFlags(1),
        [Position(0, 0, 0), Position(1, 0, 0), Position(1, 1, 0), Position(0, 1, 0)],
        [2],
        [],
        [],
    )
    area2 = NavArea(
        2,
        0,
        DynamicAttributeFlags(1),
        [Position(0, 2, 0), Position(1, 2, 0), Position(1, 3, 0), Position(0, 3, 0)],
        [],
        [],
        [],
    )
    nav = Nav(1, 2, {1: area1, 2: area2}, is_analyzed=True)
    assert nav.find_area(pos1) == area1
    assert nav.find_area(pos_1bad) is None
    assert nav.find_closest_area_centroid(pos1) == area1
    assert nav.find_closest_area_centroid(pos_1bad) == area1
    assert nav.find_path(1, 2) == PathResult(path=[area1, area2], distance=2.0)
    assert nav.find_path(pos1, pos2) == PathResult(path=[area1, area2], distance=2.0)
    assert nav.find_path(1, pos2) == PathResult(path=[area1, area2], distance=2.0)
    assert nav.find_path(pos1, 2) == PathResult(path=[area1, area2], distance=2.0)
    assert nav.find_path(2, 1).path == []


def test_nav_files() -> None:
    nav_from_bin = Nav.from_path(DATA_PATH / "de_whistle.nav")
    nav_from_json = Nav.from_json(DATA_PATH / "de_whistle.json")
    temp_path = DATA_PATH / "temp_de_whistle.nav"
    nav_from_bin.to_json(temp_path)
    nav_from_saved = Nav.from_json(temp_path)
    assert nav_from_json == nav_from_bin == nav_from_saved
    temp_path.unlink()


def test_triangle():
    pos1 = Position(1, 2, 3)
    pos2 = Position(4, 5, 6)
    pos3 = Position(7, 8, 9)
    tri = Triangle(pos1, pos2, pos3)
    assert isinstance(tri.get_centroid(), Position)


def test_visibility_checker():
    ray_origin = Position(0, 0, 0)
    ray_end = Position(1, 0, 0)
    tri1 = Triangle(Position(0.5, -1, -1), Position(0.5, 1, -1), Position(0.5, 0, -2))
    tri2 = Triangle(Position(0.5, -1, -1), Position(0.5, 1, -1), Position(0.5, 0, 1))
    checker = VisibilityChecker(triangles=[tri1])
    assert checker.n_triangles == 1
    assert checker.is_visible(ray_origin, ray_end)
    assert checker.is_visible((0, 0, 0), [1, 0, 0])
    assert tri2.ray_intersection(ray_origin, ray_end) is not None

    checker_from_tri_file = VisibilityChecker(path=DATA_PATH / "lobby_mapveto.tri")
    triangles = VisibilityChecker.read_tri_file(DATA_PATH / "lobby_mapveto.tri")
    assert isinstance(triangles, list)
    for tri in triangles:
        assert isinstance(tri, Triangle)
    checker_from_loaded_tris = VisibilityChecker(triangles=triangles)
    assert checker_from_tri_file.n_triangles == checker_from_loaded_tris.n_triangles

    with pytest.raises(ValueError, match="Exactly one of tri_file or triangles must be provided"):
        VisibilityChecker(path="test.json", triangles=[tri1])

    with pytest.raises(ValueError, match="Exactly one of tri_file or triangles must be provided"):
        VisibilityChecker()

    with pytest.raises(ValueError, match="No triangles provided"):
        VisibilityChecker(triangles=[])


def test_regularize_nav_areas() -> None:
    area1 = NavArea(
        1,
        0,
        DynamicAttributeFlags(1),
        [Position(0, 0, 0), Position(1, 0, 0), Position(1, 1, 0), Position(0, 1, 0)],
        [2],
        [],
        [],
    )
    area2 = NavArea(
        2,
        0,
        DynamicAttributeFlags(1),
        [Position(0, 2, 0), Position(1, 2, 0), Position(1, 3, 0), Position(0, 3, 0)],
        [],
        [],
        [],
    )
    nav_areas = {1: area1, 2: area2}
    vis_checker = VisibilityChecker(triangles=[Triangle(Position(1, 2, 3), Position(4, 5, 6), Position(7, 8, 9))])
    regularized = regularize_nav_areas(nav_areas, 2, vis_checker)
    assert isinstance(regularized, dict)
    for key, value in regularized.items():
        assert isinstance(key, int)
        assert isinstance(value, NavArea)


def test_group_nav_areas() -> None:
    area1 = NavArea(
        1,
        0,
        DynamicAttributeFlags(1),
        [Position(0, 0, 0), Position(1, 0, 0), Position(1, 1, 0), Position(0, 1, 0)],
        [2],
        [],
        [],
    )
    area2 = NavArea(
        2,
        0,
        DynamicAttributeFlags(1),
        [Position(0, 2, 0), Position(1, 2, 0), Position(1, 3, 0), Position(0, 3, 0)],
        [],
        [],
        [],
    )
    nav_areas = [area1, area2]
    grouped = group_nav_areas(nav_areas, 2)
    assert isinstance(grouped, dict)
    for key, value in grouped.items():
        assert isinstance(key, int)
        assert isinstance(value, int)


def test_invalid_nav_file_error() -> None:
    with tempfile.NamedTemporaryFile() as temp_file, pytest.raises(
        InvalidNavFileError, match="Could not read magic number"
    ):
        Nav.from_path(temp_file.name)
