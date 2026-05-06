"""Behavior state machine and local RRT lane-change planner.

The global planner provides reference paths for both parking-lot lanes. This
module decides when the ego vehicle should keep following the current lane,
generate an RRT Reeds-Shepp lane-change path, follow the adjacent lane, and
return to the original lane when the lead vehicle gap becomes safe again.
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Sequence

from motion_planning.path_interpolation.cubic_spline_planner import calc_spline_course

try:
    from motion_planning.local_planning.rrt_reeds_shepp import RRTReedsShepp
except ImportError:
    raise

show_animation = True
DEBUG_LOGGING = False


def debug_log(*args):
    """Print planner internals only when DEBUG_LOGGING is enabled."""
    if DEBUG_LOGGING:
        print(*args)


def smooth_rrt_path(rrt_path: Sequence[Sequence[float]]):
    """Return RRT path arrays with spline-consistent yaw and curvature."""
    raw_x = np.array([state[0] for state in rrt_path])[::-1]
    raw_y = np.array([state[1] for state in rrt_path])[::-1]
    raw_yaw = np.array([state[2] for state in rrt_path])[::-1]

    rrt_x, rrt_y = [raw_x[0]], [raw_y[0]]
    for x_value, y_value in zip(raw_x[1:], raw_y[1:]):
        if math.hypot(x_value - rrt_x[-1], y_value - rrt_y[-1]) > 0.05:
            rrt_x.append(x_value)
            rrt_y.append(y_value)
    rrt_x = np.array(rrt_x)
    rrt_y = np.array(rrt_y)

    if len(rrt_x) < 2:
        return rrt_x, rrt_y, raw_yaw, [0.0] * len(rrt_x)

    if len(rrt_x) >= 4:
        spline_x, spline_y, spline_yaw, spline_k, _ = calc_spline_course(rrt_x.tolist(), rrt_y.tolist(), ds=0.2)
        return spline_x, spline_y, spline_yaw, spline_k

    dx = np.gradient(rrt_x)
    dy = np.gradient(rrt_y)
    local_path_yaw = np.unwrap(np.arctan2(dy, dx))
    if len(local_path_yaw) >= 5:
        kernel = np.ones(5) / 5.0
        local_path_yaw = np.convolve(local_path_yaw, kernel, mode="same")

    ds = np.hypot(dx, dy)
    ds[ds < 1e-6] = 1e-6
    local_path_k = np.gradient(local_path_yaw) / ds

    return rrt_x, rrt_y, local_path_yaw.tolist(), local_path_k.tolist()


# state machine states
follow_lane = 0
find_lane_change = 1
lane_change = 2
extend_path_after_lane_change = 3
follow_second_lane = 4
find_double_lane_change = 5
double_lane_change = 6
extend_path_after_double_lane_change = 7
follow_lane_again = 8


@dataclass(frozen=True)
class BehaviourPlannerConfig:
    """Tunable thresholds used by the behavior state machine."""

    time_gap: float = 1.0
    safety_distance: float = 2.0
    vehicle_length_offset: float = 4.0
    lane_change_speed_threshold: float = 2.77
    rrt_target_index_offset: int = 100
    rrt_return_target_index_offset: int | None = None
    rrt_max_iter: int = 10
    path_end_distance_threshold: float = 0.3
    follow_lane_after_points: int = 120
    return_lane_change_min_x: float | None = None


class BehaviouralLocalPlanner:
    """Stateful local planner that owns generated RRT lane-change paths."""

    def __init__(self, cx, cy, cyaw, ck, config: BehaviourPlannerConfig | None = None):
        """Create a planner instance with optional behavior thresholds."""
        self.config = config or BehaviourPlannerConfig()
        self.follow_lead_vehicle = False
        self.lane_change_maneuver = False
        self.double_lane_change_maneuver = False
        self.lane_change_path_found = False
        self.double_lane_change_path_found = False
        self.rrt_lane_change_x = []
        self.rrt_lane_change_y = []
        self.rrt_lane_change_yaw = []
        self.rrt_lane_change_k = []
        self.rrt_lane_change_x_2 = []
        self.rrt_lane_change_y_2 = []
        self.rrt_lane_change_yaw_2 = []
        self.rrt_lane_change_k_2 = []
        self.lane_change_path_extended = False
        self.double_lane_change_path_extended = False

    def _extend_with_reference_suffix(
        self,
        path_x,
        path_y,
        path_yaw,
        path_k,
        reference_x,
        reference_y,
        reference_yaw,
        reference_k,
    ):
        """Append the closest remaining reference path points to a local RRT path."""
        if not path_x:
            return

        distances = [
            (path_x[-1] - ref_x) ** 2 + (path_y[-1] - ref_y) ** 2
            for ref_x, ref_y in zip(reference_x, reference_y)
        ]
        start_index = distances.index(min(distances))

        path_x.extend(reference_x[start_index:])
        path_y.extend(reference_y[start_index:])
        path_yaw.extend(reference_yaw[start_index:])
        path_k.extend(reference_k[start_index:])

    def local_path(self, cx, cy, cyaw, ck, cx_2, cy_2, cyaw_2, ck_2, current_state, state, obstacleList):
        """Return the currently active path for the controller to track.

        Depending on the finite-state-machine state, this can be the original
        lane path, the adjacent-lane path, or one of the generated local RRT
        lane-change paths extended with a global reference path.
        """

        if current_state == follow_lane:
            local_path_x = cx
            local_path_y = cy
            local_path_yaw = cyaw
            local_path_k = ck
            debug_log('lane change path found: ', self.lane_change_path_found)
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found

        elif current_state == find_lane_change:
            local_path_x = cx
            local_path_y = cy
            local_path_yaw = cyaw
            local_path_k = ck
            x, y, yaw, k = \
                self.rrt_lane_change_path(cx, cy, cyaw, ck, cx_2, cy_2, cyaw_2, ck_2, state, obstacleList)
            self.rrt_lane_change_x.extend(x)
            self.rrt_lane_change_y.extend(y)
            self.rrt_lane_change_yaw.extend(yaw)
            self.rrt_lane_change_k.extend(k)
            self.lane_change_path_found = True
            debug_log('lane change path found: ', self.lane_change_path_found)
            debug_log("lane change x: ", self.rrt_lane_change_x )
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found
        elif current_state == lane_change:
            self.lane_change_path_found = True
            local_path_x = self.rrt_lane_change_x
            local_path_y = self.rrt_lane_change_y
            local_path_yaw = self.rrt_lane_change_yaw
            local_path_k = self.rrt_lane_change_k
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found

        elif current_state == extend_path_after_lane_change:
            local_path_x = self.rrt_lane_change_x
            local_path_y = self.rrt_lane_change_y
            local_path_yaw = self.rrt_lane_change_yaw
            local_path_k = self.rrt_lane_change_k
            if not self.lane_change_path_extended:
                self._extend_with_reference_suffix(
                    self.rrt_lane_change_x,
                    self.rrt_lane_change_y,
                    self.rrt_lane_change_yaw,
                    self.rrt_lane_change_k,
                    cx_2,
                    cy_2,
                    cyaw_2,
                    ck_2,
                )
                self.lane_change_path_extended = True
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found

        elif current_state == follow_second_lane:
            local_path_x = self.rrt_lane_change_x
            local_path_y = self.rrt_lane_change_y
            local_path_yaw = self.rrt_lane_change_yaw
            local_path_k = self.rrt_lane_change_k
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found

        elif current_state == find_double_lane_change:
            local_path_x = self.rrt_lane_change_x
            local_path_y = self.rrt_lane_change_y
            local_path_yaw = self.rrt_lane_change_yaw
            local_path_k = self.rrt_lane_change_k
            x_2, y_2, yaw_2, k_2 = \
                self.rrt_double_lane_change_path(cx, cy, cyaw, ck, cx_2, cy_2, cyaw_2, ck_2, state, obstacleList)
            self.rrt_lane_change_x_2.extend(x_2)
            self.rrt_lane_change_y_2.extend(y_2)
            self.rrt_lane_change_yaw_2.extend(yaw_2)
            self.rrt_lane_change_k_2.extend(k_2)
            self.double_lane_change_path_found = True
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found

        elif current_state == double_lane_change:
            local_path_x = self.rrt_lane_change_x_2
            local_path_y = self.rrt_lane_change_y_2
            local_path_yaw = self.rrt_lane_change_yaw_2
            local_path_k = self.rrt_lane_change_k_2
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found

        elif current_state == extend_path_after_double_lane_change:
            local_path_x = self.rrt_lane_change_x_2
            local_path_y = self.rrt_lane_change_y_2
            local_path_yaw = self.rrt_lane_change_yaw_2
            local_path_k = self.rrt_lane_change_k_2
            if not self.double_lane_change_path_extended:
                self._extend_with_reference_suffix(
                    self.rrt_lane_change_x_2,
                    self.rrt_lane_change_y_2,
                    self.rrt_lane_change_yaw_2,
                    self.rrt_lane_change_k_2,
                    cx,
                    cy,
                    cyaw,
                    ck,
                )
                self.double_lane_change_path_extended = True
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found

        elif current_state == follow_lane_again:
            local_path_x = self.rrt_lane_change_x_2
            local_path_y = self.rrt_lane_change_y_2
            local_path_yaw = self.rrt_lane_change_yaw_2
            local_path_k = self.rrt_lane_change_k_2
            return local_path_x, local_path_y, local_path_yaw, local_path_k, self.lane_change_path_found
        else:
            raise ValueError('cannot find a state')

    def path_found(self):
        """Report whether the first lane-change RRT path has been generated."""
        if not self.lane_change_path_found:
            return False
        return True

    def transition_state(self, state, state2, cx, cy, cyaw, ck, cx_2, cy_2, cyaw_2, ck_2, obstacleList):
        """Evaluate distances and path progress to choose the next behavior state."""

        current_state = follow_lane

        self.lane_change_maneuver = self.check_for_lane_change(state, state2)
        if self.lane_change_maneuver and not self.lane_change_path_found:
            current_state = find_lane_change

        if len(self.rrt_lane_change_x) > 2:
            current_state = lane_change

        if self.rrt_lane_change_x:
            debug_log("number of points change path", len(self.rrt_lane_change_x))
            rrt_x_end_point = self.rrt_lane_change_x[- 1]
            rrt_y_end_point = self.rrt_lane_change_y[- 1]
            debug_log("rrt_x_end_point: ", rrt_x_end_point)
            dx_1 = state.x - rrt_x_end_point  # distance between the current state and the last point of the rrt path
            dy_1 = state.y - rrt_y_end_point  # in order to switch to the other lane
            distance_between_state_and_path = math.hypot(dx_1, dy_1)
            debug_log("distance_between_state_and_path: ", distance_between_state_and_path)
            if distance_between_state_and_path < self.config.path_end_distance_threshold:
                current_state = extend_path_after_lane_change

        if len(self.rrt_lane_change_x) > self.config.follow_lane_after_points:
            current_state = follow_second_lane

        self.double_lane_change_maneuver = self.check_for_double_lane_change(state, state2)
        if self.double_lane_change_maneuver and not self.double_lane_change_path_found:
            current_state = find_double_lane_change

        if len(self.rrt_lane_change_x_2) > 2:
            current_state = double_lane_change

        if self.rrt_lane_change_x_2:
            debug_log("number of points change path 2", len(self.rrt_lane_change_x_2))
            rrt_x_end_point_2 = self.rrt_lane_change_x_2[- 1]
            rrt_y_end_point_2 = self.rrt_lane_change_y_2[- 1]
            debug_log("rrt_x_end_point: ", rrt_x_end_point_2)
            dx_2 = state.x - rrt_x_end_point_2  # distance between the current state and the last point of the rrt path
            dy_2 = state.y - rrt_y_end_point_2  # in order to switch to the other lane
            distance_between_state_and_path_2 = math.hypot(dx_2, dy_2)
            debug_log("distance_between_state_and_path 2: ", distance_between_state_and_path_2)
            if distance_between_state_and_path_2 < self.config.path_end_distance_threshold:
                current_state = extend_path_after_double_lane_change

        if len(self.rrt_lane_change_x_2) > self.config.follow_lane_after_points:
            current_state = follow_lane_again
        debug_log('path found: ', self.lane_change_path_found)
        debug_log('current state: ', current_state)
        return current_state




    def check_for_lane_change(self, state, state2):
        """Return True when the lead vehicle is close and slow enough to overtake."""
        dx1 = state.x - state2.x
        dy1 = state.y - state2.y
        # distance between the front of the ego vehicle and the rear of the leading vehicle
        lead_car_distance = math.hypot(dx1, dy1) - self.config.vehicle_length_offset

        distance_gap = (self.config.time_gap * state.v) + self.config.safety_distance

        if lead_car_distance > distance_gap:
            self.lane_change_maneuver = False

        if lead_car_distance < distance_gap and state2.v < self.config.lane_change_speed_threshold and len(self.rrt_lane_change_x) < 2:
            self.lane_change_maneuver = True

        return self.lane_change_maneuver

    def check_for_double_lane_change(self, state, state2):
        """Return True when there is enough gap to return to the original lane."""
        if self.config.return_lane_change_min_x is not None and state.x < self.config.return_lane_change_min_x:
            return False

        dx2 = state.x - state2.x
        dy2 = state.y - state2.y
        # distance between the front of the ego vehicle and the rear of the leading vehicle
        lead_car_distance = math.hypot(dx2, dy2) - self.config.vehicle_length_offset

        distance_gap = self.config.time_gap * state.v

        if len(self.rrt_lane_change_x) > self.config.follow_lane_after_points and lead_car_distance > distance_gap:
            self.double_lane_change_maneuver = True

        return self.double_lane_change_maneuver

    def rrt_lane_change_path(self, cx, cy, cyaw, ck, cx_2, cy_2, cyaw_2, ck_2, state, obstacleList):
        """Plan the first RRT Reeds-Shepp path from the ego pose to the adjacent lane."""
        start_rrt = [state.x, state.y, state.yaw]
        debug_log('start of rrt: ', start_rrt)
        target_ind2, mind = calc_nearest_index(state, cx_2, cy_2,
                                               cyaw_2)  # curretn index and distance to index on the other lane planned path
        target_ind_distance = target_ind2 + self.config.rrt_target_index_offset
        debug_log('current target index on the left lane: ', target_ind2)
        goal_rrt = [cx_2[target_ind_distance], cy_2[target_ind_distance],
                    cyaw_2[target_ind_distance]]
        debug_log('goal of rrt: ', goal_rrt)

        rrt_reeds_shepp = RRTReedsShepp(start_rrt, goal_rrt, obstacleList, [0.0, 40.0],
                                        max_iter=self.config.rrt_max_iter)
        rrt_path = rrt_reeds_shepp.planning(animation=False)
        if rrt_path is None:
            raise RuntimeError("Cannot find collision-free lane-change RRT path")

        local_path_x, local_path_y, local_path_yaw, local_path_k = smooth_rrt_path(rrt_path)
        debug_log('curvature', local_path_k)

        return local_path_x, local_path_y, local_path_yaw, local_path_k

    def rrt_double_lane_change_path(self, cx, cy, cyaw, ck, cx_2, cy_2, cyaw_2, ck_2, state, obstacleList):
        """Plan the return RRT Reeds-Shepp path back to the original lane."""
        start_rrt2 = [state.x, state.y, state.yaw]
        debug_log('start of rrt_2: ', start_rrt2)
        target_ind2, mind = calc_nearest_index(state, cx, cy,
                                               cyaw)  # current index on the original right lane planned path
        target_offset = self.config.rrt_return_target_index_offset or self.config.rrt_target_index_offset
        target_ind_distance = target_ind2 + target_offset
        goal_rrt2 = [cx[target_ind_distance], cy[target_ind_distance],
                     cyaw[target_ind_distance]]
        debug_log('goal of rrt_2: ', goal_rrt2)

        rrt_reeds_shepp_2 = RRTReedsShepp(start_rrt2, goal_rrt2, obstacleList, [0.0, 40.0],
                                          max_iter=self.config.rrt_max_iter)
        rrt_path_2 = rrt_reeds_shepp_2.planning(animation=False)
        if rrt_path_2 is None:
            raise RuntimeError("Cannot find collision-free return RRT path")

        local_path_x, local_path_y, local_path_yaw, local_path_k = smooth_rrt_path(rrt_path_2)

        return local_path_x, local_path_y, local_path_yaw, local_path_k


def pi_2_pi(angle):                       ### keep heading withing [-pi, pi] so optimizer behaves well
    """Normalize an angle to the [-pi, pi] interval."""
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def calc_nearest_index(state, cx, cy, cyaw):    # calculating nearest index on the planned path next to the ego car (ind)
    """Find the nearest path point and signed lateral error for a vehicle state."""
    dx = [state.x - icx for icx in cx]          # calculating also the distance between ego and closest planned point on path (e or mind)
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind                 # calculating the target index and the e(distance between the path and the tracked path)
