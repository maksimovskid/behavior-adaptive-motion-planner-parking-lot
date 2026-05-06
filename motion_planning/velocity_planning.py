"""Shared speed-profile helpers for scenario scripts."""

from __future__ import annotations

import math
from typing import Sequence

from vehicle.kinematics import calc_distance


def calc_speed_profile(
    cx: Sequence[float],
    cy: Sequence[float],
    cyaw: Sequence[float],
    target_speed: float,
    *,
    min_speed: float = 0.277,
    acceleration: float = 2.0,
    path_step: float = 0.1,
    deceleration_mode: str = "ramp",
    debug_log=None,
) -> list[float]:
    """Build a target-speed profile along a Hybrid A* path."""
    speed_profile = [target_speed] * len(cx)
    direction = 1.0

    for i in range(len(cx) - 1):
        dyaw = cyaw[i + 1] - cyaw[i]
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        speed_profile[i] = target_speed if direction == 1.0 else -target_speed

        if switch:
            speed_profile[i] = 0.0

    decel_distance = calc_distance(target_speed, min_speed, -acceleration)
    decel_points = round(decel_distance / path_step)
    stop_index = len(cx) - 1

    if debug_log is not None:
        debug_log("stop index: ", stop_index)
        debug_log("deceleration distance: ", decel_points * path_step, "(m)")

    if deceleration_mode == "ramp":
        current_speed = target_speed
        for i in range(max(0, stop_index - decel_points), stop_index):
            current_speed = math.sqrt((current_speed**2) + 2 * (-acceleration) * path_step)
            speed_profile[i] = max(current_speed, min_speed)
        speed_profile[-1] = min_speed
    elif deceleration_mode == "flat":
        for i in range(max(0, stop_index - decel_points), stop_index):
            speed_profile[i] = min_speed
        speed_profile[-1] = min_speed
    elif deceleration_mode == "none":
        pass
    else:
        raise ValueError(f"unknown deceleration_mode: {deceleration_mode}")

    return speed_profile


def slow_near_goal_speed_profile(
    cx: Sequence[float],
    speed_profile: list[float],
    target_ind: int,
    target_speed: float,
    *,
    slow_speed: float = 0.833,
    acceleration: float = 2.0,
    path_step: float = 0.1,
    buffer_points: int = 8,
) -> list[float]:
    """Reduce the remaining target speed when the vehicle is close to the goal."""
    decel_distance = calc_distance(target_speed, slow_speed, -acceleration)
    decel_points = round(decel_distance / path_step) + buffer_points

    if (len(cx) - 1) - target_ind <= decel_points:
        for i in range(len(cx) - 1):
            speed_profile[i] = slow_speed

    speed_profile[-1] = 0.0
    return speed_profile


def calc_lane_change_ego_speed_profile(
    cx: Sequence[float],
    cy: Sequence[float],
    cyaw: Sequence[float],
    target_speed: float,
    state,
    target_ind: int,
    *,
    slow_speed: float = 0.833,
    acceleration: float = 2.0,
    path_step: float = 0.1,
    goal_buffer_points: int = 10,
    current_speed_buffer_points: int = 12,
    adaptive_deceleration_min_points: int = 350,
) -> list[float]:
    """Build the ego speed profile used during lane-change tracking."""
    speed_profile = [target_speed] * len(cx)
    direction = 1.0

    for i in range(len(cx) - 1):
        dyaw = cyaw[i + 1] - cyaw[i]
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        speed_profile[i] = target_speed if direction == 1.0 else -target_speed

        if switch:
            speed_profile[i] = 0.0

    decel_distance = calc_distance(target_speed, slow_speed, -acceleration)
    decel_points = round(decel_distance / path_step) + goal_buffer_points

    if len(cx) > adaptive_deceleration_min_points:
        for i in range(len(cx) - 1):
            if ((len(cx) - 1) - target_ind) <= decel_points:
                current_decel_distance = calc_distance(state.v, slow_speed, -acceleration)
                current_decel_points = round(current_decel_distance / path_step) + current_speed_buffer_points

                if ((len(cx) - 1) - target_ind) <= current_decel_points:
                    speed_profile[i] = slow_speed
            speed_profile[-1] = slow_speed

    return speed_profile


def calc_lead_vehicle_ego_speed_profile(
    cx: Sequence[float],
    cy: Sequence[float],
    cyaw: Sequence[float],
    target_speed: float,
    ego_state,
    lead_state,
    target_ind: int,
    *,
    slow_speed: float = 0.833,
    acceleration: float = 2.0,
    path_step: float = 0.1,
    time_gap: float = 1.0,
    safety_distance: float = 2.0,
    vehicle_length_offset: float = 4.0,
    lookahead_distance: float = 15.0,
    goal_buffer_points: int = 8,
    current_speed_buffer_points: int = 12,
) -> list[float]:
    """Build the ego speed profile for scenarios with a lead vehicle."""
    speed_profile = [target_speed] * len(cx)
    direction = 1.0

    for i in range(len(cx) - 1):
        dyaw = cyaw[i + 1] - cyaw[i]
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        speed_profile[i] = target_speed if direction == 1.0 else -target_speed

        if switch:
            speed_profile[i] = 0.0

    decel_distance = calc_distance(target_speed, slow_speed, -acceleration)
    decel_points = round(decel_distance / path_step) + goal_buffer_points

    dx = ego_state.x - lead_state.x
    dy = ego_state.y - lead_state.y
    lead_car_distance = math.hypot(dx, dy) - vehicle_length_offset
    distance_gap = (time_gap * ego_state.v) + safety_distance
    distance_to_vehicle_threshold = distance_gap + lookahead_distance

    for i in range(len(cx) - 1):
        if lead_car_distance <= safety_distance:
            speed_profile[i] = 0.0
        elif lead_car_distance <= distance_gap:
            speed_profile[i] = min(target_speed, lead_state.v)
        elif lead_car_distance <= distance_to_vehicle_threshold:
            speed_profile[i] = min(target_speed, lead_state.v)

        if ((len(cx) - 1) - target_ind) <= decel_points:
            current_decel_distance = calc_distance(ego_state.v, slow_speed, -acceleration)
            current_decel_points = round(current_decel_distance / path_step) + current_speed_buffer_points

            if ((len(cx) - 1) - target_ind) <= current_decel_points:
                speed_profile[i] = slow_speed

    speed_profile[-1] = slow_speed
    return speed_profile
