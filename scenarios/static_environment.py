"""Static-environment scenario with one ego vehicle.

The scenario plans a Hybrid A* route through the parking lot, generates a speed
profile, tracks the path with the shared controllers, and creates the static
summary figure or live/GIF visualization.
"""
import matplotlib.pyplot as plt
import numpy as np

from motion_planning.velocity_planning import calc_speed_profile, slow_near_goal_speed_profile
from simulation.parking_lot_map import build_parking_lot_obstacles
from simulation.path_setup import prepare_hybrid_path
from simulation.plotting import latest_colored_pose, plot_parking_lot_layout, plot_planning_scene, plot_speed_profile, plot_tracking_summary
from simulation.tracking import simulate_path_tracking
from vehicle.car import plot_car
from vehicle.controllers import make_pid_controller

XY_GRID_RESOLUTION = 1.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0)  # [rad]


# steering control parameter

dt = 0.1  # [s]

PIDControl = make_pid_controller(kp=10.0)
show_animation = False
DEBUG_LOGGING = False


def debug_log(*args):
    """Print scenario diagnostics only when DEBUG_LOGGING is enabled."""
    if DEBUG_LOGGING:
        print(*args)
#  show_animation = False


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal, target_speed):     #closed loop prediction
    """Track the static Hybrid A* path and optionally draw live frames."""
    live_x = [cx[0]]
    live_y = [cy[0]]

    def update_profile(profile, target_ind, time, state):
        """Update the speed profile near the goal during simulation."""
        return speed_profile_update(cx, profile, target_ind, target_speed, time)

    def plot_step(state, target_ind, time):
        """Draw one live frame with reference path and tracked history."""
        if target_ind % 1 == 0 and show_animation:
            live_x.append(state.x)
            live_y.append(state.y)
            plt.cla()
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plot_parking_lot_layout()
            plt.plot(cx, cy, color="0.75", linestyle="--", linewidth=1.0, label="Hybrid A* path")
            plt.plot(live_x, live_y, "-g", linewidth=2.0, label="tracked trajectory")
            plt.plot(state.x, state.y, "ob", label="ego")
            plot_car(state.x, state.y, state.yaw, color="b")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(False)
            plt.title(f"t = {time:.1f} s, speed: {state.v * 3.6:.1f} km/h, target index: {target_ind}")
            plt.pause(0.0001)

    result = simulate_path_tracking(
        cx,
        cy,
        cyaw,
        ck,
        speed_profile,
        goal,
        PIDControl,
        dt=dt,
        speed_profile_updater=update_profile,
        step_callback=plot_step if show_animation else None,
        debug_log=debug_log,
    )
    return result.t, result.x, result.y, result.yaw, result.v, result.goal_reached

def speed_profile_update(cx, speed_profile, target_ind, target_speed, t):
    """Slow the vehicle near the final goal."""
    return slow_near_goal_speed_profile(cx, speed_profile, target_ind, target_speed)

def main():
    """Run the static parking-lot scenario."""
    print("simulation start")
    ox, oy = build_parking_lot_obstacles()

    # Set Initial parameters
    start = [1.5, 8, np.deg2rad(0.0)]
    goal = [13.0, 32.0, np.deg2rad(180.0)]

    plot_planning_scene(ox, oy, [start, goal])

    ego_path = prepare_hybrid_path(start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
    cx, cy, cyaw, ck = ego_path.x, ego_path.y, ego_path.yaw, ego_path.curvature
    debug_log('length of path: ', ego_path.length_m, '(m)')

    target_speed = 15.0 / 3.6


    sp = calc_speed_profile(cx, cy, cyaw, target_speed, deceleration_mode="none", debug_log=debug_log)
    debug_log('speed profile:', sp)
    debug_log('speed profile points:', len(sp))


    # ols = open_loop_speed(sp, dt)    # ols - open loop speed

    t, x, y, yaw, v, goal_flag = closed_loop_prediction(
        cx, cy, cyaw, ck, sp, goal, target_speed)

    print(f'simulation end: Goal reached, total time: {len(t) * 0.1:.1f} s')
    debug_log('velocity profile: ', v)    # velocity profile at each timestep dt=0.1 (s)
    debug_log('velocity points: ', len(v))
    # Test
    assert goal_flag, "Cannot goal"

    plot_tracking_summary(
        ox,
        oy,
        courses=[(cx, cy, "--", "Hybrid A* path")],
        trajectories=[(x, y, "-g", "tracked trajectory")],
        vehicle_poses=[latest_colored_pose(x, y, yaw, "b")],
        target_pose=goal,
    )

    if show_animation:  # pragma: no cover
        plot_speed_profile(t, v)
        plt.show()

if __name__ == '__main__':
    main()
