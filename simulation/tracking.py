"""Closed-loop path tracking simulation helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence

from vehicle.controllers import calc_nearest_index, rear_wheel_feedback_control
from vehicle.kinematics import State, update


@dataclass(frozen=True)
class TrackingResult:
    """Time history returned by a closed-loop tracking simulation."""

    t: list[float]
    x: list[float]
    y: list[float]
    yaw: list[float]
    v: list[float]
    goal_reached: bool


SpeedProfileUpdater = Callable[[list[float], int, float, State], list[float]]
StepCallback = Callable[[State, int, float], None]


def simulate_path_tracking(
    cx: Sequence[float],
    cy: Sequence[float],
    cyaw: Sequence[float],
    ck: Sequence[float],
    speed_profile: list[float],
    goal: Sequence[float],
    pid_control: Callable[[float, float], float],
    *,
    initial_yaw: float = 0.0,
    max_time: float = 500.0,
    dt: float = 0.1,
    goal_distance: float = 0.3,
    stop_speed: float = 0.05,
    speed_profile_updater: SpeedProfileUpdater | None = None,
    step_callback: StepCallback | None = None,
    debug_log=None,
) -> TrackingResult:
    """Track a path with rear-wheel feedback steering and PID speed control."""
    state = State(x=cx[0], y=cy[0], yaw=initial_yaw, v=0.0)
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    goal_reached = False
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw)

    while max_time >= time:
        di, target_ind = rear_wheel_feedback_control(state, cx, cy, cyaw, ck, target_ind)
        current_profile = speed_profile
        if speed_profile_updater is not None:
            current_profile = speed_profile_updater(speed_profile, target_ind, time, state)

        ai = pid_control(current_profile[target_ind], state.v)
        state = update(state, ai, di)

        if debug_log is not None:
            debug_log("speed profile at target index: ", current_profile[target_ind])

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time += dt

        if math.hypot(state.x - goal[0], state.y - goal[1]) <= goal_distance:
            goal_reached = True
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if step_callback is not None:
            step_callback(state, target_ind, time)

    return TrackingResult(t=t, x=x, y=y, yaw=yaw, v=v, goal_reached=goal_reached)
