"""Simple vehicle state and kinematic update used by thesis scenarios."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class State:
    """Mutable vehicle state used by the closed-loop simulations."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0


def update(state: State, a: float, delta: float, dt: float = 0.1, wheel_base: float = 2.7) -> State:
    """Advance the vehicle state using the thesis bicycle model."""

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / wheel_base * math.tan(delta) * dt
    state.v = state.v + a * dt
    return state


def calc_distance(initial_speed: float, final_speed: float, acceleration: float) -> float:
    """Distance needed to change speed with constant acceleration."""

    return (final_speed * final_speed - initial_speed * initial_speed) / (2.0 * acceleration)


def calc_time(initial_speed: float, final_speed: float, acceleration: float) -> float:
    """Time needed to change speed with constant acceleration."""

    return (final_speed - initial_speed) / acceleration


def calc_final_speed(initial_speed: float, acceleration: float, distance: float) -> float:
    """Final speed after driving a distance with constant acceleration."""

    speed_squared = initial_speed * initial_speed + 2.0 * distance * acceleration
    if speed_squared < 0.0:
        return 0.000001
    return math.sqrt(speed_squared)
