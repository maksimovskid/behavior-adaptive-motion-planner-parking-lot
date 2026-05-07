"""Shared lane-change scenario engine.

The small lane-change scenario wrappers provide only configuration values.
This module builds the parking-lot map, prepares Hybrid A* reference paths,
runs the behavior/local RRT planner, tracks both vehicles, and optionally draws
or records the live visualization.
"""

import matplotlib.pyplot as plt
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Sequence

from motion_planning.velocity_planning import calc_lane_change_ego_speed_profile, calc_speed_profile
from vehicle.car import plot_car
from vehicle.controllers import calc_nearest_index, make_pid_controller, rear_wheel_feedback_control
from vehicle.kinematics import State, update
from simulation.parking_lot_map import build_parking_lot_obstacles, build_rrt_obstacle_list, build_vehicle_obstacle_points
from simulation.path_setup import prepare_hybrid_path
from simulation.plotting import latest_colored_pose, plot_parking_lot_layout, plot_planning_scene, plot_speed_profile, plot_tracking_summary

from motion_planning.local_planning.behaviour_and_local_planner import (
    BehaviouralLocalPlanner,
    BehaviourPlannerConfig,
    double_lane_change,
    lane_change,
)
from scenarios.config_loader import behavior_config_from_dict, load_config


XY_GRID_RESOLUTION = 1.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0)  # [rad]


dt = 0.1  # [s]

PIDControl = make_pid_controller(kp=10.0)
show_animation = True
DEBUG_LOGGING = False


def debug_log(*args):
    """Print scenario diagnostics only when DEBUG_LOGGING is enabled."""
    if DEBUG_LOGGING:
        print(*args)


def minimum_distance_to_points(x_values: Sequence[float], y_values: Sequence[float], points: Sequence[tuple[float, float]]) -> float:
    """Return the minimum center-point distance from a trajectory to obstacle points."""
    return min(
        math.hypot(x_value - obstacle_x, y_value - obstacle_y)
        for x_value, y_value in zip(x_values, y_values)
        for obstacle_x, obstacle_y in points
    )


def draw_live_frame(
    cx: Sequence[float],
    cy: Sequence[float],
    cx2: Sequence[float],
    cy2: Sequence[float],
    local_path_x: Sequence[float],
    local_path_y: Sequence[float],
    x: Sequence[float],
    y: Sequence[float],
    x2: Sequence[float],
    y2: Sequence[float],
    state,
    state2,
    target_ind2: int,
    local_path_found: bool,
    target_pose: Sequence[float],
    time: float,
    obstacle_label: str,
) -> None:
    """Draw one live animation frame for the lane-change scenarios."""
    plt.cla()
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    plot_parking_lot_layout(target_pose=target_pose)
    plt.plot(cx, cy, color="0.75", linestyle="--", linewidth=1.0, label="reference ego")
    plt.plot(cx2, cy2, color="0.78", linestyle=":", linewidth=1.0, label=f"reference {obstacle_label}")
    plt.plot(x, y, "-g", linewidth=2.0, label="trajectory ego")
    plt.plot(x2, y2, "-c", linewidth=1.8, label=f"trajectory {obstacle_label}")
    if local_path_found and len(local_path_x) > 2:
        plt.plot(local_path_x, local_path_y, color="limegreen", linestyle="-", linewidth=2.4, alpha=0.85, label="active local path")
    plt.plot(state.x, state.y, "ob", label="ego")
    plt.plot(state2.x, state2.y, "or", label=obstacle_label)
    if obstacle_label != "static obstacle":
        plt.plot(cx2[target_ind2], cy2[target_ind2], "xr", label=f"{obstacle_label} target")
    plot_car(state.x, state.y, state.yaw, color="b")
    plot_car(state2.x, state2.y, state2.yaw, color="r")
    plt.axis("equal")
    plt.grid(False)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(f"t = {time:.1f} s, ego: {state.v * 3.6:.1f} km/h, {obstacle_label}: {state2.v * 3.6:.1f} km/h")
    plt.legend(loc="center")
    plt.pause(0.0001)


@dataclass(frozen=True)
class LaneChangeScenarioConfig:
    """Configuration values that distinguish one lane-change variant."""

    name: str
    goal_x: float
    lead_start_x: float
    lead_initial_speed: float
    lead_target_speed: float
    ego_target_speed: float = 15.0 / 3.6
    obstacle_label: str = "lead"
    add_stopped_vehicle_to_rrt: bool = False
    behavior: BehaviourPlannerConfig = field(default_factory=BehaviourPlannerConfig)


def load_lane_change_config(name: str) -> LaneChangeScenarioConfig:
    """Load a lane-change scenario config from scenarios/configs."""
    data = load_config(name)
    return LaneChangeScenarioConfig(
        name=data["name"],
        goal_x=data["goal_x"],
        lead_start_x=data["lead_start_x"],
        lead_initial_speed=data["lead_initial_speed"],
        lead_target_speed=data["lead_target_speed_kmh"] / 3.6,
        ego_target_speed=data.get("ego_target_speed_kmh", 15.0) / 3.6,
        obstacle_label=data.get("obstacle_label", "lead"),
        add_stopped_vehicle_to_rrt=data.get("add_stopped_vehicle_to_rrt", False),
        behavior=behavior_config_from_dict(data.get("behavior")),
    )


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile_ego,  goal, cx2, cy2, cyaw2, ck2, cx_2, cy_2, cyaw_2, ck_2,
                           speed_profile2, goal2, target_speed, obstacleList, ox, oy,
                           lead_initial_speed=0.1, behavior_config=None, obstacle_label="lead"):
    """Run the closed-loop two-vehicle lane-change simulation."""

    T = 500.0  # max simulation time
    goal_dis = 0.4
    stop_speed = 0.05

    state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)
    state2 = State(x=cx2[0], y=cy2[0], yaw=cyaw2[0], v=lead_initial_speed)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    goal_flag = False
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw)

    x2 = [state2.x]
    y2 = [state2.y]
    yaw2 = [state2.yaw]
    v2 = [state2.v]
    t2 = [0.0]
    goal_flag2 = False
    target_ind2, _ = calc_nearest_index(state2, cx2, cy2, cyaw2)
    previous_behavior_state = None
    path_planning = BehaviouralLocalPlanner(cx, cy, cyaw, ck, config=behavior_config)

    while T >= time:
        target_ind = min(target_ind, len(cx) - 1)
        target_ind2 = min(target_ind2, len(cx2) - 1)
        current_state = path_planning.transition_state(state, state2, cx, cy, cyaw, ck, cx_2, cy_2, cyaw_2, ck_2, obstacleList)
        if current_state != previous_behavior_state:
            if current_state == lane_change:
                print(f"lane change start: t = {time:.1f} s")
            elif previous_behavior_state == lane_change:
                print(f"lane change end: t = {time:.1f} s")
            elif current_state == double_lane_change:
                print(f"return lane change start: t = {time:.1f} s")
            elif previous_behavior_state == double_lane_change:
                print(f"return lane change end: t = {time:.1f} s")
        previous_behavior_state = current_state
        local_path_x, local_path_y, local_path_yaw, local_path_k, boolean = path_planning.local_path \
            (cx, cy, cyaw, ck, cx_2, cy_2, cyaw_2, ck_2, current_state, state, obstacleList)
        cx_local_path, cy_local_path, cyaw_local_path, ck_local_path = local_path_x, local_path_y, local_path_yaw, local_path_k
        target_ind = min(target_ind, len(cx_local_path) - 1)
        di, target_ind = rear_wheel_feedback_control(
            state, cx_local_path, cy_local_path, cyaw_local_path, ck_local_path)
        di2, target_ind2 = rear_wheel_feedback_control(
            state2, cx2, cy2, cyaw2, ck2, target_ind2)
        speed_profile_ego = calc_lane_change_ego_speed_profile(
            local_path_x,
            local_path_y,
            local_path_yaw,
            target_speed,
            state,
            target_ind,
        )
        ai = PIDControl(speed_profile_ego[target_ind], state.v)
        ai2 = PIDControl(speed_profile2[target_ind2], state2.v)
        state = update(state, ai, di)
        state2 = update(state2, ai2, di2)
        if abs(state.v) <= stop_speed:
            target_ind += 1
        if abs(state2.v) <= stop_speed:
            target_ind2 += 1
        target_ind = min(target_ind, len(cx_local_path) - 1)
        target_ind2 = min(target_ind2, len(cx2) - 1)

        time = time + dt

        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            debug_log("Goal reached")
            goal_flag = True
            break

        dx2 = state2.x - goal2[0]
        dy2 = state2.y - goal2[1]
        if math.hypot(dx2, dy2) <= goal_dis:
            goal_flag2 = True


        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        x2.append(state2.x)
        y2.append(state2.y)
        yaw2.append(state2.yaw)
        v2.append(state2.v)
        t2.append(time)

        if show_animation:
            draw_live_frame(
                cx,
                cy,
                cx2,
                cy2,
                local_path_x,
                local_path_y,
                x,
                y,
                x2,
                y2,
                state,
                state2,
                target_ind2,
                boolean,
                goal,
                time,
                obstacle_label,
            )

    return t, x, y, yaw, v, goal_flag, t2, x2, y2, yaw2, v2, goal_flag2


def run_scenario(config: LaneChangeScenarioConfig) -> None:
    """Prepare paths, execute a configured lane-change scenario, and plot results."""
    print("simulation start")
    ox, oy = build_parking_lot_obstacles(
        exit_open=True,
        middle_vertical_start=15,
        middle_vertical_stop=26,
        middle_lower_stop=33,
    )
    obstacle_list = build_rrt_obstacle_list()

    start = [1.5, 8, np.deg2rad(0.0)]
    goal = [config.goal_x, 32.0, np.deg2rad(180.0)]
    start_2 = [1.5, 12.0, np.deg2rad(0.0)]
    goal_2 = [config.goal_x, 28.0, np.deg2rad(180.0)]
    start2 = [config.lead_start_x, 8.0, np.deg2rad(0.0)]
    goal2 = [0.0, 32.0, np.deg2rad(180)]
    stopped_vehicle_obstacles = []
    if config.add_stopped_vehicle_to_rrt:
        stopped_vehicle_obstacles = build_vehicle_obstacle_points(start2[0], start2[1])
        obstacle_list.extend(stopped_vehicle_obstacles)

    plot_planning_scene(ox, oy, [start, goal, start2, goal2], target_pose=goal)

    ego_path = prepare_hybrid_path(start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
    adjacent_lane_path = prepare_hybrid_path(start_2, goal_2, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
    lead_path = prepare_hybrid_path(start2, goal2, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

    cx, cy, cyaw, ck = ego_path.x, ego_path.y, ego_path.yaw, ego_path.curvature
    cx_2, cy_2, cyaw_2, ck_2 = (
        adjacent_lane_path.x,
        adjacent_lane_path.y,
        adjacent_lane_path.yaw,
        adjacent_lane_path.curvature,
    )
    cx2, cy2, cyaw2, ck2 = lead_path.x, lead_path.y, lead_path.yaw, lead_path.curvature
    debug_log('length of path: ', ego_path.length_m, '(m)')

    target_speed = config.ego_target_speed
    target_speed2 = config.lead_target_speed

    sp = calc_speed_profile(cx, cy, cyaw, target_speed, deceleration_mode="flat", debug_log=debug_log)
    debug_log('speed profile:', sp)
    debug_log('speed profile points:', len(sp))

    sp2 = calc_speed_profile(cx2, cy2, cyaw2, target_speed2, deceleration_mode="flat", debug_log=debug_log)
    t, x, y, yaw, v, goal_flag, t2, x2, y2, yaw2, v2, goal_flag2 = closed_loop_prediction(
        cx, cy, cyaw, ck, sp, goal, cx2, cy2, cyaw2, ck2, cx_2, cy_2, cyaw_2,
        ck_2, sp2, goal2, target_speed, obstacle_list, ox, oy, config.lead_initial_speed, config.behavior,
        config.obstacle_label)

    status = "Goal reached" if goal_flag else "Goal not reached"
    print(f'simulation end: {status}, total time: {len(t) * 0.1:.1f} s')
    debug_log('velocity profile: ', v)
    debug_log('velocity points: ', len(v))
    if stopped_vehicle_obstacles:
        clearance = minimum_distance_to_points(x, y, stopped_vehicle_obstacles)
        assert clearance > 1.2, f"Ego trajectory too close to stopped obstacle: {clearance:.2f} m"
    assert goal_flag, "Cannot find goal"

    if show_animation:  # pragma: no cover
        plot_tracking_summary(
            ox,
            oy,
            courses=[],
            trajectories=[(x, y, "-g", "trajectory ego")],
            vehicle_poses=[latest_colored_pose(x, y, yaw, "b")],
            target_pose=goal,
        )
        plot_speed_profile(t, v, secondary_time=t2, secondary_velocity=v2, secondary_label=config.obstacle_label)
        plt.show()


