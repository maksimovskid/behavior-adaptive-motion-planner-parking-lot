"""Regenerate the curated PNG and GIF media used by the README."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]

COMMANDS = [
    [
        sys.executable,
        "scripts/run_scenario.py",
        "static_environment",
        "--timeout",
        "300",
        "--save-plot",
        "docs/figures/static_environment_planned_and_tracked_trajectory.png",
    ],
    [
        sys.executable,
        "scripts/run_scenario.py",
        "static_environment",
        "--timeout",
        "300",
        "--save-gif",
        "docs/figures/static_environment_live_planning_and_tracking.gif",
        "--gif-frame-stride",
        "4",
        "--gif-max-frames",
        "120",
        "--gif-dpi",
        "80",
    ],
    [
        sys.executable,
        "scripts/run_scenario.py",
        "lane_change_3",
        "--timeout",
        "300",
        "--save-plot",
        "docs/figures/lane_change_3_velocity_profiles.png",
    ],
    [
        sys.executable,
        "scripts/run_scenario.py",
        "lane_change_3",
        "--timeout",
        "300",
        "--save-gif",
        "docs/figures/lane_change_3_dynamic_obstacle_live.gif",
        "--gif-frame-stride",
        "4",
        "--gif-max-frames",
        "140",
        "--gif-dpi",
        "80",
    ],
]


def main() -> int:
    """Run the scenario commands that produce README media files."""
    for command in COMMANDS:
        print(f"\n> {' '.join(command)}", flush=True)
        completed = subprocess.run(command, cwd=REPO_ROOT)
        if completed.returncode != 0:
            return completed.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
