"""Shared longitudinal and lateral controllers for scenario simulations."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence


DEFAULT_REAR_WHEEL_HEADING_GAIN = 1.0
DEFAULT_REAR_WHEEL_LATERAL_GAIN = 0.7


def make_pid_controller(kp: float = 10.0, max_accel: float = 2.0) -> Callable[[float, float], float]:
    """Create a proportional longitudinal speed controller.

    The thesis scenarios only used the proportional term, despite the old
    function name `PIDControl`.
    """

    def controller(target: float, current: float) -> float:
        """Return saturated acceleration for one target/current speed pair."""
        return longitudinal_pid_control(target, current, kp=kp, max_accel=max_accel)

    return controller


def longitudinal_pid_control(
    target: float,
    current: float,
    kp: float = 10.0,
    max_accel: float = 2.0,
) -> float:
    """Longitudinal speed controller with acceleration saturation."""

    accel = kp * (target - current)
    return max(-max_accel, min(max_accel, accel))


def pi_2_pi(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def rear_wheel_feedback_control(
    state,
    cx: Sequence[float],
    cy: Sequence[float],
    cyaw: Sequence[float],
    ck: Sequence[float],
    preind=None,
    kth: float = DEFAULT_REAR_WHEEL_HEADING_GAIN,
    ke: float = DEFAULT_REAR_WHEEL_LATERAL_GAIN,
    wheel_base: float = 2.7,
    max_steer: float = 0.6,
    epsilon: float = 1e-6,
) -> tuple[float, int]:
    """Rear-wheel feedback lateral controller from the thesis scenarios.

    `kth` damps heading error; `ke` corrects lateral error. A slightly lower
    lateral gain reduces post-lane-change steering chatter in the parking-lot
    scenarios while keeping the original thesis controller structure.
    """

    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    if abs(v) < epsilon or abs(th_e) < epsilon:
        return 0.0, ind

    omega = (
        v * k * math.cos(th_e) / (1.0 - k * e)
        - kth * abs(v) * th_e
        - ke * v * math.sin(th_e) * e / th_e
    )

    if abs(omega) < epsilon:
        return 0.0, ind

    delta = math.atan2(wheel_base * omega / v, 1.0)
    return min(delta, max_steer), ind


def calc_nearest_index(state, cx: Sequence[float], cy: Sequence[float], cyaw: Sequence[float]) -> tuple[int, float]:
    """Find the closest path index and signed lateral distance."""

    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind
