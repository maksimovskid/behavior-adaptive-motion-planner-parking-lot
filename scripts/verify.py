"""Run the standard local verification checks."""

from __future__ import annotations

import subprocess
import sys


COMMANDS = [
    [sys.executable, "scripts/check_compile.py"],
    [sys.executable, "-m", "unittest", "discover", "-s", "tests"],
    [sys.executable, "scripts/run_scenario.py", "static_environment", "--timeout", "300", "--json"],
    [sys.executable, "scripts/run_scenario.py", "lead_vehicle_following", "--timeout", "300", "--json"],
    [sys.executable, "scripts/run_scenario.py", "static_obstacle_avoidance", "--timeout", "300", "--json"],
    [sys.executable, "scripts/run_scenario.py", "lane_change_3", "--timeout", "300", "--json"],
]


def main() -> int:
    """Run each verification command and stop on the first failure."""
    for command in COMMANDS:
        print(f"\n> {' '.join(command)}", flush=True)
        completed = subprocess.run(command)
        if completed.returncode != 0:
            return completed.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
