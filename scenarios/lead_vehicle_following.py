"""Lead-vehicle following scenario.

The ego vehicle follows a Hybrid A* path and adapts its velocity to a slower
vehicle ahead while both vehicles are tracked in the same simulation loop.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import matplotlib.pyplot as plt
import numpy as np

from motion_planning.velocity_planning import calc_lead_vehicle_ego_speed_profile, calc_speed_profile
from scenarios.config_loader import load_config, pose_from_degrees
from simulation.parking_lot_map import build_parking_lot_obstacles
from simulation.path_setup import PreparedPath, prepare_hybrid_path
from simulation.plotting import (
    latest_colored_pose,
    plot_parking_lot_layout,
    plot_planning_scene,
    plot_speed_profile,
    plot_tracking_summary,
)
from vehicle.car import plot_car
from vehicle.controllers import calc_nearest_index, make_pid_controller, rear_wheel_feedback_control
from vehicle.kinematics import State, update


XY_GRID_RESOLUTION = 1.0
YAW_GRID_RESOLUTION = np.deg2rad(15.0)
dt = 0.1

PIDControl = make_pid_controller(kp=10.0)
show_animation = False
DEBUG_LOGGING = False


def debug_log(*args):
    """Print scenario diagnostics only when DEBUG_LOGGING is enabled."""
    if DEBUG_LOGGING:
        print(*args)


@dataclass(frozen=True)
class LeadVehicleScenarioConfig:
    """Configuration for a two-vehicle lead-following scenario."""

    ego_start: list[float]
    ego_goal: list[float]
    lead_start: list[float]
    lead_goal: list[float]
    ego_target_speed: float
    lead_target_speed: float
    lead_initial_speed: float
    obstacle_options: dict = field(default_factory=lambda: {"exit_open": True})


def default_config() -> LeadVehicleScenarioConfig:
    """Return the thesis lead-following scenario defaults."""
    return load_lead_vehicle_config("lead_vehicle_following")


def load_lead_vehicle_config(name: str) -> LeadVehicleScenarioConfig:
    """Load a lead-following scenario config from scenarios/configs."""
    data = load_config(name)
    return LeadVehicleScenarioConfig(
        ego_start=pose_from_degrees(data["ego_start"]),
        ego_goal=pose_from_degrees(data["ego_goal"]),
        lead_start=pose_from_degrees(data["lead_start"]),
        lead_goal=pose_from_degrees(data["lead_goal"]),
        ego_target_speed=data["ego_target_speed_kmh"] / 3.6,
        lead_target_speed=data["lead_target_speed_kmh"] / 3.6,
        lead_initial_speed=data["lead_initial_speed"],
        obstacle_options=data.get("obstacle_options", {"exit_open": True}),
    )


def closed_loop_prediction(
    ego_path: PreparedPath,
    lead_path: PreparedPath,
    ego_speed_profile: list[float],
    lead_speed_profile: list[float],
    goal: list[float],
    goal2: list[float],
    target_speed: float,
    lead_initial_speed: float,
):
    """Run both vehicles in one loop so ego speed adapts to the lead vehicle."""
    max_time = 500.0
    goal_distance = 0.3
    stop_speed = 0.05

    state = State(x=ego_path.x[0], y=ego_path.y[0], yaw=0.0, v=0.0)
    state2 = State(x=lead_path.x[0], y=lead_path.y[0], yaw=0.0, v=lead_initial_speed)

    time = 0.0
    x, y, yaw, v, t = [state.x], [state.y], [state.yaw], [state.v], [0.0]
    x2, y2, yaw2, v2, t2 = [state2.x], [state2.y], [state2.yaw], [state2.v], [0.0]
    goal_flag = False
    goal_flag2 = False

    target_ind, _ = calc_nearest_index(state, ego_path.x, ego_path.y, ego_path.yaw)
    target_ind2, _ = calc_nearest_index(state2, lead_path.x, lead_path.y, lead_path.yaw)

    while max_time >= time:
        di, target_ind = rear_wheel_feedback_control(
            state, ego_path.x, ego_path.y, ego_path.yaw, ego_path.curvature, target_ind
        )
        di2, target_ind2 = rear_wheel_feedback_control(
            state2, lead_path.x, lead_path.y, lead_path.yaw, lead_path.curvature, target_ind2
        )

        ego_speed_profile = calc_lead_vehicle_ego_speed_profile(
            ego_path.x, ego_path.y, ego_path.yaw, target_speed, state, state2, target_ind
        )
        ai = PIDControl(ego_speed_profile[target_ind], state.v)
        ai2 = PIDControl(lead_speed_profile[target_ind2], state2.v)

        state = update(state, ai, di)
        state2 = update(state2, ai2, di2)

        debug_log("speed profile at target index: ", ego_speed_profile[target_ind])

        if abs(state.v) <= stop_speed:
            target_ind += 1
        if abs(state2.v) <= stop_speed:
            target_ind2 += 1

        time += dt

        if math.hypot(state.x - goal[0], state.y - goal[1]) <= goal_distance:
            goal_flag = True
            break
        if math.hypot(state2.x - goal2[0], state2.y - goal2[1]) <= goal_distance:
            goal_flag2 = True
            break

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
            plt.cla()
            plt.gcf().canvas.mpl_connect(
                "key_release_event", lambda event: [exit(0) if event.key == "escape" else None]
            )
            plot_parking_lot_layout()
            plt.plot(ego_path.x, ego_path.y, color="0.75", linestyle="--", linewidth=1.0, label="reference ego")
            plt.plot(lead_path.x, lead_path.y, color="0.78", linestyle=":", linewidth=1.0, label="reference lead")
            plt.plot(x, y, "-g", linewidth=2.0, label="trajectory ego")
            plt.plot(x2, y2, "-c", linewidth=1.8, label="trajectory 2")
            plt.plot(state.x, state.y, "ob", label="ego")
            plt.plot(state2.x, state2.y, "or", label="lead")
            plot_car(state.x, state.y, state.yaw, color="b")
            plot_car(state2.x, state2.y, state2.yaw, color="r")
            plt.plot(ego_path.x[target_ind], ego_path.y[target_ind], "xg", label="ego target")
            plt.plot(lead_path.x[target_ind2], lead_path.y[target_ind2], "xr", label="lead target")
            plt.axis("equal")
            plt.grid(False)
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.title(f"t = {time:.1f} s, ego {state.v * 3.6:.1f} km/h, lead {state2.v * 3.6:.1f} km/h")
            plt.legend(loc="center")
            plt.pause(0.0001)

    return t, x, y, yaw, v, goal_flag, t2, x2, y2, yaw2, v2, goal_flag2


def run_scenario(config: LeadVehicleScenarioConfig | None = None) -> None:
    """Prepare paths, execute the lead-following scenario, and plot results."""
    config = config or default_config()
    print("simulation start")
    ox, oy = build_parking_lot_obstacles(**config.obstacle_options)

    plot_planning_scene(ox, oy, [config.ego_start, config.ego_goal, config.lead_start, config.lead_goal])

    ego_path = prepare_hybrid_path(
        config.ego_start, config.ego_goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION
    )
    lead_path = prepare_hybrid_path(
        config.lead_start, config.lead_goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION
    )
    debug_log("length of path: ", ego_path.length_m, "(m)")

    ego_speed_profile = calc_speed_profile(
        ego_path.x,
        ego_path.y,
        ego_path.yaw,
        config.ego_target_speed,
        deceleration_mode="flat",
        debug_log=debug_log,
    )
    lead_speed_profile = calc_speed_profile(
        lead_path.x,
        lead_path.y,
        lead_path.yaw,
        config.lead_target_speed,
        deceleration_mode="flat",
        debug_log=debug_log,
    )

    t, x, y, yaw, v, goal_flag, t2, x2, y2, yaw2, v2, goal_flag2 = closed_loop_prediction(
        ego_path,
        lead_path,
        ego_speed_profile,
        lead_speed_profile,
        config.ego_goal,
        config.lead_goal,
        config.ego_target_speed,
        config.lead_initial_speed,
    )

    status = "Goal reached" if goal_flag else "Goal not reached"
    print(f"simulation end: {status}, total time: {len(t) * dt:.1f} s")
    debug_log("velocity points: ", len(v))
    assert goal_flag, "Cannot find goal"

    if show_animation:  # pragma: no cover
        plot_tracking_summary(
            ox,
            oy,
            courses=[],
            trajectories=[(x, y, "-g", "trajectory ego"), (x2, y2, "-c", "trajectory 2")],
            vehicle_poses=[latest_colored_pose(x, y, yaw, "b"), latest_colored_pose(x2, y2, yaw2, "r")],
        )
        plot_speed_profile(t, v, secondary_time=t2, secondary_velocity=v2)
        plt.show()


def main() -> None:
    """Run the default lead-vehicle following configuration."""
    run_scenario()


if __name__ == "__main__":
    main()
