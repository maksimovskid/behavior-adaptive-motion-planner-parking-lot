"""Lane-change scenario with the longer goal path from the thesis experiments."""

from __future__ import annotations

from scenarios.lane_change import load_lane_change_config, run_scenario


EXTENDED_GOAL_CONFIG = load_lane_change_config("lane_change_3")


def main():
    """Run the extended-goal lane-change configuration."""
    run_scenario(EXTENDED_GOAL_CONFIG)


if __name__ == "__main__":
    main()
