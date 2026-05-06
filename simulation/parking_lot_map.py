"""Shared parking-lot obstacle definitions used by thesis scenarios."""

from __future__ import annotations


Point = tuple[float, float]


def _append_horizontal(ox: list[float], oy: list[float], y: float, start: int, stop: int) -> None:
    """Append obstacle points along one horizontal grid line."""
    for x in range(start, stop):
        ox.append(float(x))
        oy.append(float(y))


def _append_vertical(ox: list[float], oy: list[float], x: float, start: int, stop: int) -> None:
    """Append obstacle points along one vertical grid line."""
    for y in range(start, stop):
        ox.append(float(x))
        oy.append(float(y))


def build_parking_lot_obstacles(
    *,
    exit_open: bool = False,
    middle_vertical_start: int = 14,
    middle_vertical_stop: int = 27,
    middle_lower_stop: int = 34,
) -> tuple[list[float], list[float]]:
    """Build the Hybrid A* obstacle grid for the parking-lot scenarios.

    The keyword arguments preserve the small differences between the original
    copied scripts.
    """

    ox: list[float] = []
    oy: list[float] = []

    _append_horizontal(ox, oy, 0.0, 0, 42)
    _append_vertical(ox, oy, 41.0, 0, 41)
    _append_horizontal(ox, oy, 40.0, 0, 42)

    if exit_open:
        _append_vertical(ox, oy, 0.0, 0, 26)
        _append_vertical(ox, oy, 0.0, 34, 42)
    else:
        _append_vertical(ox, oy, 0.0, 0, 41)

    _append_horizontal(ox, oy, 0.0, 4, 33)
    _append_vertical(ox, oy, 33.0, middle_vertical_start, middle_vertical_stop)
    _append_horizontal(ox, oy, 26.0, 0, 33)
    _append_horizontal(ox, oy, 14.0, 0, middle_lower_stop)

    _append_horizontal(ox, oy, 6.0, 0, 42)
    _append_horizontal(ox, oy, 34.0, 0, 37)
    _append_vertical(ox, oy, 37.0, 34, 41)
    _append_horizontal(ox, oy, 10.0, 0, 37)
    _append_vertical(ox, oy, 37.0, 11, 30)
    _append_horizontal(ox, oy, 30.0, 0, 37)

    return ox, oy


def build_rrt_obstacle_list() -> list[Point]:
    """Build the compact RRT obstacle list used by overtake/lane-change scripts."""

    obstacles: list[Point] = []

    obstacles.extend((x, 6) for x in range(0, 42))
    obstacles.extend((41, y) for y in range(6, 41))
    obstacles.extend((x, 39) for x in range(40, 36, -1))
    obstacles.extend((37, y) for y in range(38, 33, -1))
    obstacles.extend((x, 34) for x in range(36, -1, -1))
    obstacles.extend((0, y) for y in range(33, -1, -1))
    obstacles.extend((x, 14) for x in range(0, 33))
    obstacles.extend((33, y) for y in range(15, 26))
    obstacles.extend((x, 26) for x in range(32, -1, -1))
    obstacles.extend((x, 0) for x in range(0, 7))
    obstacles.extend((x, 34) for x in range(38, 42))

    return obstacles


def build_vehicle_obstacle_points(
    rear_x: float,
    rear_y: float,
    *,
    rear_overhang: float = 1.0,
    front_overhang: float = 3.0,
    half_width: float = 1.0,
    point_step: float = 1.0,
) -> list[Point]:
    """Approximate a stopped vehicle footprint as RRT obstacle points."""
    points: list[Point] = []
    x = rear_x - rear_overhang
    while x <= rear_x + front_overhang:
        points.append((round(x, 3), round(rear_y - half_width, 3)))
        points.append((round(x, 3), round(rear_y + half_width, 3)))
        x += point_step
    y = rear_y - half_width
    while y <= rear_y + half_width:
        points.append((round(rear_x - rear_overhang, 3), round(y, 3)))
        points.append((round(rear_x + front_overhang, 3), round(y, 3)))
        y += point_step
    return points
