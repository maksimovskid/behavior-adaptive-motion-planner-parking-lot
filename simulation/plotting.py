"""Shared plotting helpers for scenario scripts."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from motion_planning.global_planning import reeds_shepp_path_planning as rs
from vehicle.car import plot_car


Pose = Sequence[float]
Point = tuple[float, float]
LANE_MARKING_COLOR = "#d6a600"


@dataclass(frozen=True)
class LineSegment:
    """Straight map feature used for walls and lane markings."""

    start: Point
    end: Point


@dataclass(frozen=True)
class CurveLine:
    """Curved map feature sampled as connected points."""

    points: tuple[Point, ...]


@dataclass(frozen=True)
class ParkingSpace:
    """Parking-bay rectangle used only for semantic visualization."""

    x: float
    y: float
    width: float
    length: float
    occupied: bool = False


@dataclass(frozen=True)
class ParkingLotLayout:
    """Renderable parking-lot layout independent of obstacle-point generation."""

    walls: tuple[LineSegment, ...]
    lane_markings: tuple[LineSegment, ...]
    curved_lane_markings: tuple[CurveLine, ...]
    parking_spaces: tuple[ParkingSpace, ...]


def _make_row(
    *,
    x_start: float,
    y: float,
    count: int,
    width: float = 2.5,
    length: float = 4.4,
    gap: float = 0.15,
    occupied: bool = False,
) -> tuple[ParkingSpace, ...]:
    """Create a row of equal-sized parking spaces for the semantic map."""
    return tuple(
        ParkingSpace(
            x_start + index * (width + gap),
            y,
            width,
            length,
            occupied,
        )
        for index in range(count)
    )


def _smooth_middle_lane_marking() -> CurveLine:
    """Create a continuous U-shaped lane divider with rounded corner markings."""
    points: list[Point] = [(0.0, 10.0), (35.0, 10.0)]
    radius = 2.0

    for index in range(1, 13):
        angle = -math.pi / 2.0 + (math.pi / 2.0) * index / 12.0
        points.append((35.0 + radius * math.cos(angle), 12.0 + radius * math.sin(angle)))

    points.extend([(37.0, 28.0)])

    for index in range(1, 13):
        angle = (math.pi / 2.0) * index / 12.0
        points.append((35.0 + radius * math.cos(angle), 28.0 + radius * math.sin(angle)))

    points.append((0.0, 30.0))
    return CurveLine(tuple(points))


DEFAULT_LAYOUT = ParkingLotLayout(
    walls=(
        LineSegment((0, 0), (41, 0)),
        LineSegment((41, 0), (41, 40)),
        LineSegment((0, 40), (41, 40)),
        LineSegment((0, 0), (0, 26)),
        LineSegment((0, 34), (0, 40)),
        LineSegment((33, 14), (33, 26)),
        LineSegment((0, 14), (33, 14)),
        LineSegment((0, 26), (33, 26)),
        LineSegment((0, 6), (41, 6)),
        LineSegment((0, 34), (41, 34)),
    ),
    lane_markings=(),
    curved_lane_markings=(_smooth_middle_lane_marking(),),
    parking_spaces=(
        *_make_row(x_start=1.4, y=1.0, count=14),
        *_make_row(x_start=1.4, y=15.0, count=12),
        *_make_row(x_start=1.4, y=21.0, count=12),
        *_make_row(x_start=1.4, y=34.8, count=7),
        *_make_row(x_start=20.5, y=34.8, count=7),
    ),
)


def _plot_segments(
    segments: Iterable[LineSegment],
    *,
    color: str,
    linewidth: float,
    linestyle: str = "-",
    alpha: float = 1.0,
) -> None:
    """Draw line segments with one shared style."""
    for segment in segments:
        plt.plot(
            [segment.start[0], segment.end[0]],
            [segment.start[1], segment.end[1]],
            color=color,
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alpha,
        )


def _plot_curves(
    curves: Iterable[CurveLine],
    *,
    color: str,
    linewidth: float,
    linestyle: str = "-",
    alpha: float = 1.0,
) -> None:
    """Draw sampled curved map markings with one shared style."""
    for curve in curves:
        x_values = [point[0] for point in curve.points]
        y_values = [point[1] for point in curve.points]
        plt.plot(x_values, y_values, color=color, linewidth=linewidth, linestyle=linestyle, alpha=alpha)


def _target_spot_for_goal(goal_pose: Pose | None) -> ParkingSpace | None:
    """Choose the top-row parking bay closest to a scenario goal pose."""
    if goal_pose is None:
        return None

    top_spaces = [space for space in DEFAULT_LAYOUT.parking_spaces if space.y >= 34.0]
    goal_x = goal_pose[0]
    return min(top_spaces, key=lambda space: abs((space.x + space.width / 2.0) - goal_x))


def _plot_parking_spaces(spaces: Iterable[ParkingSpace], *, target_space: ParkingSpace | None = None) -> None:
    """Draw parking-space rectangles and optionally highlight the target bay."""
    ax = plt.gca()
    for space in spaces:
        facecolor = "#f3f4f6"
        edgecolor = "#b8bec7"
        linewidth = 0.9
        if space.occupied:
            facecolor = "#ef4444"
            edgecolor = "#b91c1c"
        elif target_space == space:
            facecolor = "#dcfce7"
            edgecolor = "#16a34a"
            linewidth = 1.6
        ax.add_patch(
            Rectangle(
                (space.x, space.y),
                space.width,
                space.length,
                facecolor=facecolor,
                edgecolor=edgecolor,
                linewidth=linewidth,
                zorder=0,
            )
        )


def _format_parking_axes() -> None:
    """Apply consistent axis limits, aspect ratio, and grid styling."""
    plt.xlim(-1, 43)
    plt.ylim(-1, 42)
    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid(True, color="#e5e7eb", linewidth=0.6)


def plot_parking_lot_layout(
    *,
    layout: ParkingLotLayout = DEFAULT_LAYOUT,
    target_pose: Pose | None = None,
    show_spaces: bool = True,
    show_lane_markings: bool = True,
) -> None:
    """Draw the parking lot as a human-readable map, not obstacle dots."""

    target_space = _target_spot_for_goal(target_pose)
    if show_spaces:
        _plot_parking_spaces(layout.parking_spaces, target_space=target_space)
    _plot_segments(layout.walls, color="black", linewidth=1.9)
    if show_lane_markings:
        _plot_segments(layout.lane_markings, color=LANE_MARKING_COLOR, linewidth=1.1, linestyle="--", alpha=0.9)
        _plot_curves(layout.curved_lane_markings, color=LANE_MARKING_COLOR, linewidth=1.1, linestyle="--", alpha=0.9)
    _format_parking_axes()


def plot_planning_scene(
    ox: Sequence[float],
    oy: Sequence[float],
    poses: Iterable[Pose],
    *,
    target_pose: Pose | None = None,
) -> None:
    """Plot the parking-lot layout and start/goal arrows before planning."""

    plot_parking_lot_layout(target_pose=target_pose)
    for index, pose in enumerate(poses):
        kwargs = {"fc": "g"} if index % 2 == 0 else {}
        rs.plot_arrow(pose[0], pose[1], pose[2], **kwargs)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")


def plot_speed_profile(
    time: Sequence[float],
    velocity: Sequence[float],
    *,
    secondary_time: Sequence[float] | None = None,
    secondary_velocity: Sequence[float] | None = None,
    primary_label: str = "ego vehicle",
    secondary_label: str = "lead vehicle",
) -> None:
    """Plot velocity profiles in km/h."""

    plt.subplots(1)
    plt.plot(time, [value * 3.6 for value in velocity], "-r", label=primary_label)
    if secondary_time is not None and secondary_velocity is not None:
        plt.plot(secondary_time, [value * 3.6 for value in secondary_velocity], "-b", label=secondary_label)
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [km/h]")
    plt.grid(True)
    plt.legend()


def plot_tracking_summary(
    ox: Sequence[float],
    oy: Sequence[float],
    *,
    courses: Sequence[tuple[Sequence[float], Sequence[float], str, str]],
    trajectories: Sequence[tuple[Sequence[float], Sequence[float], str, str]],
    vehicle_poses: Iterable[tuple[float, float, float] | tuple[float, float, float, str]] = (),
    target_pose: Pose | None = None,
) -> None:
    """Plot planned paths, tracked trajectories, and optional vehicles."""

    plt.close()
    plt.subplots(1)
    plot_parking_lot_layout(target_pose=target_pose)
    for x_values, y_values, style, label in courses:
        plt.plot(x_values, y_values, style, label=label)
    for x_values, y_values, style, label in trajectories:
        plt.plot(x_values, y_values, style, label=label)
    for pose in vehicle_poses:
        if len(pose) == 4:
            x, y, yaw, color = pose
        else:
            x, y, yaw = pose
            color = "b"
        plot_car(x, y, yaw, color=color)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend(loc="center")


def latest_pose(
    x_values: Sequence[float],
    y_values: Sequence[float],
    yaw_values: Sequence[float],
) -> tuple[float, float, float]:
    """Return the last vehicle pose from a trajectory history."""

    return x_values[-1], y_values[-1], yaw_values[-1]


def latest_colored_pose(
    x_values: Sequence[float],
    y_values: Sequence[float],
    yaw_values: Sequence[float],
    color: str,
) -> tuple[float, float, float, str]:
    """Return the last vehicle pose with a plotting color."""

    return x_values[-1], y_values[-1], yaw_values[-1], color
