"""Static-obstacle avoidance scenario.

The ego vehicle encounters a stopped obstacle in its lane and uses the shared
lane-change/RRT behavior planner to overtake it and return toward the goal lane.
"""

from scenarios.lane_change import load_lane_change_config, run_scenario


STATIC_OBSTACLE_CONFIG = load_lane_change_config("static_obstacle_avoidance")


def main():
    """Run the stopped-obstacle avoidance configuration."""
    run_scenario(STATIC_OBSTACLE_CONFIG)


if __name__ == "__main__":
    main()
