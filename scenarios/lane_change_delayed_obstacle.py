"""Lane-change scenario with the lead vehicle starting farther ahead."""

from __future__ import annotations

from scenarios.lane_change import load_lane_change_config, run_scenario


DELAYED_OBSTACLE_CONFIG = load_lane_change_config("lane_change_2")


def main():
    """Run the delayed-obstacle lane-change configuration."""
    run_scenario(DELAYED_OBSTACLE_CONFIG)


if __name__ == "__main__":
    main()
