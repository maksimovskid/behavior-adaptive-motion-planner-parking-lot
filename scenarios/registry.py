"""Scenario registry for the thesis simulation scripts.

The registry gives stable, descriptive names to the original script files
without renaming the thesis artifacts yet.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Scenario:
    """Metadata needed to run one original thesis scenario script."""

    name: str
    script: str
    thesis_group: str
    description: str
    expected_goal_message: str = "Goal reached"
    timeout_seconds: int = 240

    @property
    def script_path(self) -> Path:
        """Absolute path to the scenario script on disk."""
        return REPO_ROOT / self.script


SCENARIOS: dict[str, Scenario] = {
    "static_environment": Scenario(
        name="static_environment",
        script="scenarios/static_environment.py",
        thesis_group="5.1 Scenarios in a Static Environment",
        description="Hybrid A* global path, velocity profile, and controller tracking without a dynamic obstacle.",
    ),
    "lead_vehicle_following": Scenario(
        name="lead_vehicle_following",
        script="scenarios/lead_vehicle_following.py",
        thesis_group="5.2 Scenarios with Lead Vehicle",
        description="Ego vehicle adapts its speed to a moving vehicle ahead.",
    ),
    "static_obstacle_avoidance": Scenario(
        name="static_obstacle_avoidance",
        script="scenarios/static_obstacle_avoidance.py",
        thesis_group="5.3 Static Obstacle Avoidance",
        description="Ego vehicle overtakes a stopped obstacle using the shared lane-change/RRT planner.",
    ),
    "lane_change_1": Scenario(
        name="lane_change_1",
        script="scenarios/lane_change_close_obstacle.py",
        thesis_group="5.3/5.4 Lane Change Maneuvers",
        description="Lane-change maneuver variant 1 from the original thesis workspace.",
    ),
    "lane_change_2": Scenario(
        name="lane_change_2",
        script="scenarios/lane_change_delayed_obstacle.py",
        thesis_group="5.3/5.4 Lane Change Maneuvers",
        description="Lane-change maneuver variant 2 from the original thesis workspace.",
    ),
    "lane_change_3": Scenario(
        name="lane_change_3",
        script="scenarios/lane_change_extended_goal.py",
        thesis_group="5.3/5.4 Lane Change Maneuvers",
        description="Lane-change maneuver variant 3 from the original thesis workspace.",
    ),
}


def get_scenario(name: str) -> Scenario:
    """Look up a registered scenario by stable CLI name."""
    try:
        return SCENARIOS[name]
    except KeyError as exc:
        available = ", ".join(sorted(SCENARIOS))
        raise KeyError(f"Unknown scenario '{name}'. Available scenarios: {available}") from exc
