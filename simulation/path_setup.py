"""Shared Hybrid A* path preparation for scenario scripts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from motion_planning.global_planning import hybrid_a_star
from motion_planning.path_interpolation.cubic_spline_planner import Spline2D


@dataclass(frozen=True)
class PreparedPath:
    """Hybrid A* path arrays plus curvature prepared for closed-loop tracking."""

    x: list[float]
    y: list[float]
    yaw: list[float]
    curvature: list[float]
    length_m: float


def prepare_hybrid_path(
    start: Sequence[float],
    goal: Sequence[float],
    ox: Sequence[float],
    oy: Sequence[float],
    xy_grid_resolution: float,
    yaw_grid_resolution: float,
    *,
    interpolation_step: float = 0.1,
) -> PreparedPath:
    """Plan a Hybrid A* path and compute spline curvature for tracking."""

    previous_animation = hybrid_a_star.show_animation
    hybrid_a_star.show_animation = False
    try:
        path = hybrid_a_star.hybrid_a_star_planning(start, goal, ox, oy, xy_grid_resolution, yaw_grid_resolution)
    finally:
        hybrid_a_star.show_animation = previous_animation
    x = path.xlist
    y = path.ylist
    yaw = path.yawlist
    spline = Spline2D(x, y)
    s_values = np.arange(0, spline.s[-1], interpolation_step)
    curvature = [spline.calc_curvature(i_s) for i_s in s_values]
    if len(curvature) < len(x):
        curvature.extend([curvature[-1]] * (len(x) - len(curvature)))
    elif len(curvature) > len(x):
        curvature = curvature[: len(x)]
    return PreparedPath(
        x=x,
        y=y,
        yaw=yaw,
        curvature=curvature,
        length_m=(len(x) - 1) * interpolation_step,
    )
