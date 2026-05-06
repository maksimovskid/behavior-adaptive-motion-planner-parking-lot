"""Lane-change scenario with a close slow lead vehicle."""

from __future__ import annotations

from scenarios.lane_change import load_lane_change_config, run_scenario


CLOSE_OBSTACLE_CONFIG = load_lane_change_config("lane_change_1")


def main():
    """Run the close-obstacle lane-change configuration."""
    run_scenario(CLOSE_OBSTACLE_CONFIG)


if __name__ == "__main__":
    main()
