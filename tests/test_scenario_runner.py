"""Tests for scenario registry metadata and CLI runner behavior."""

from __future__ import annotations

import os
import subprocess
import sys
import unittest

from scenarios.registry import SCENARIOS


class ScenarioRegistryTests(unittest.TestCase):
    """Coverage for registered scenario scripts and runner entry points."""

    def test_registered_scripts_exist(self) -> None:
        """Every registered scenario should point to an existing script."""
        missing = [scenario.script for scenario in SCENARIOS.values() if not scenario.script_path.exists()]
        self.assertEqual(missing, [])

    def test_cli_lists_scenarios(self) -> None:
        """The scenario CLI should list stable scenario names."""
        completed = subprocess.run(
            [sys.executable, "scripts/run_scenario.py", "--list"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("static_environment", completed.stdout)
        self.assertIn("lead_vehicle_following", completed.stdout)

    def test_compile_check_excludes_known_incomplete_files(self) -> None:
        """The compile-check script should succeed on active Python files."""
        completed = subprocess.run(
            [sys.executable, "scripts/check_compile.py"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("compiled:", completed.stdout)

    @unittest.skipUnless(
        os.environ.get("RUN_SLOW_SCENARIO_SMOKE") == "1",
        "set RUN_SLOW_SCENARIO_SMOKE=1 to run slow thesis scenario smoke tests",
    )
    def test_selected_scenarios_reach_goal(self) -> None:
        """Optional slow smoke test for selected scenario simulations."""
        scenario_names = os.environ.get("RUN_SCENARIO_SMOKE_NAMES", "static_environment")
        for scenario_name in [name.strip() for name in scenario_names.split(",") if name.strip()]:
            with self.subTest(scenario=scenario_name):
                completed = subprocess.run(
                    [
                        sys.executable,
                        "scripts/run_scenario.py",
                        scenario_name,
                        "--timeout",
                        "300",
                        "--json",
                    ],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                self.assertEqual(completed.returncode, 0, completed.stderr)
                self.assertIn('"goal_reached": true', completed.stdout)


if __name__ == "__main__":
    unittest.main()
