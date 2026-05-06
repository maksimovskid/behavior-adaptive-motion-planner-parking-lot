"""Load scenario configuration from JSON files."""

from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path
from typing import Any

import numpy as np

from motion_planning.local_planning.behaviour_and_local_planner import BehaviourPlannerConfig


CONFIG_DIR = Path(__file__).resolve().parent / "configs"


def load_config(name: str) -> dict[str, Any]:
    """Load a scenario JSON config by filename without extension."""
    path = CONFIG_DIR / f"{name}.json"
    with path.open(encoding="utf-8") as file:
        return json.load(file)


def pose_from_degrees(values: list[float]) -> list[float]:
    """Convert [x, y, yaw_deg] JSON pose values to [x, y, yaw_rad]."""
    return [values[0], values[1], np.deg2rad(values[2])]


def behavior_config_from_dict(values: dict[str, Any] | None) -> BehaviourPlannerConfig:
    """Create a behavior config while ignoring absent optional values."""
    if not values:
        return BehaviourPlannerConfig()

    allowed = {field.name for field in fields(BehaviourPlannerConfig)}
    return BehaviourPlannerConfig(**{key: value for key, value in values.items() if key in allowed})
