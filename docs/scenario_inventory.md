# Scenario Inventory

This document maps the thesis scenario families to the registered scenario scripts.

## Registered Scenarios

| Name | Script | Purpose |
| --- | --- | --- |
| `static_environment` | `scenarios/static_environment.py` | Single ego vehicle follows a Hybrid A* path through the parking lot. |
| `lead_vehicle_following` | `scenarios/lead_vehicle_following.py` | Ego vehicle adapts its speed behind a lead vehicle. |
| `static_obstacle_avoidance` | `scenarios/static_obstacle_avoidance.py` | Ego vehicle avoids a stopped obstacle ahead using the shared lane-change/RRT runner. |
| `lane_change_1` | `scenarios/lane_change_close_obstacle.py` | Lane-change/overtake with a close slow obstacle. |
| `lane_change_2` | `scenarios/lane_change_delayed_obstacle.py` | Lane-change/overtake with the lead vehicle farther ahead. |
| `lane_change_3` | `scenarios/lane_change_extended_goal.py` | Lane-change/overtake with a longer target path. |

## Scenario Families

### Static Environment

Purpose: validate global Hybrid A* planning, velocity planning, and closed-loop tracking without a dynamic obstacle.

Main scenario:

- `static_environment`

### Lead Vehicle Following

Purpose: validate ego speed adaptation when another vehicle is ahead on the route.

Main scenarios:

- `lead_vehicle_following`

`lead_vehicle_following` is the single retained scenario for this family. Older near-duplicate variants were removed because they did not add distinct behavior beyond lead-vehicle following.

### Lane Change And Overtaking

Purpose: validate behavior switching from lane following into an RRT Reeds-Shepp lane change, then back to the original lane when possible.

Main scenarios:

- `static_obstacle_avoidance`
- `lane_change_1`
- `lane_change_2`
- `lane_change_3`

`static_obstacle_avoidance`, `lane_change_1`, `lane_change_2`, and `lane_change_3` use the same shared lane-change/RRT engine.

Runtime sequence for the lane-change scenarios:

1. The ego and lead vehicle start on the lower lane.
2. Hybrid A* plans the reference paths through the parking lot.
3. The behavior planner monitors the lead vehicle distance and speed.
4. When overtaking is needed, RRT Reeds-Shepp generates a local lane-change path.
5. The ego tracks the active local path into the adjacent lane.
6. When the return maneuver is available, a second RRT path brings the ego back toward the goal lane.
7. The scenario ends when the ego reaches the configured goal pose.

## Scenario Config Files

Most scenario parameters now live in JSON files under `scenarios/configs/`.
The Python files in `scenarios/` are thin entry points for the runner.

Use the config files for ordinary variants. Create a new Python implementation only when the simulation loop or behavior logic changes.

## Lane-Change Parameters

The static-obstacle and lane-change variants share the same simulation implementation in `scenarios/lane_change.py`.

| Scenario | Ego Goal X | Lead Start X | Lead Initial Speed | Lead Target Speed |
| --- | ---: | ---: | ---: | ---: |
| `static_obstacle_avoidance` | `17.0 m` | `24.0 m` | `0.0 m/s` | `0.0 km/h` |
| `lane_change_1` | `5.0 m` | `9.0 m` | `0.1 m/s` | `0.1 km/h` |
| `lane_change_2` | `5.0 m` | `15.0 m` | `2.0 m/s` | `6.2 km/h` |
| `lane_change_3` | `17.0 m` | `9.0 m` | `3.61 m/s` | `2.5 km/h` |

`static_obstacle_avoidance` represents a stopped obstacle in the ego lane. Its stopped-vehicle footprint is inserted into the RRT obstacle set and the executed ego trajectory is checked for clearance. The lane-change variants represent slow or moving lead vehicles.

The visualization highlights the top-row parking bay nearest the active scenario goal. For example, `lane_change_1` and `lane_change_2` highlight a nearer free bay, while `lane_change_3` highlights the farther free bay.

## Creating New Scenarios

Use the existing scenario types before copying a whole script.

### New Lane-Change Variant

For a new lane-change/overtaking variant:

1. Create a JSON config in `scenarios/configs/`.
2. Create a small wrapper file in `scenarios/`.
3. Load the config with `load_lane_change_config`.
4. Add the scenario to `scenarios/registry.py`.
5. Run the scenario with `scripts/run_scenario.py`.

Example:

```python
from scenarios.lane_change import load_lane_change_config, run_scenario


CUSTOM_CONFIG = load_lane_change_config("lane_change_custom")


def main():
    run_scenario(CUSTOM_CONFIG)


if __name__ == "__main__":
    main()
```

Then register it:

```python
"lane_change_custom": Scenario(
    name="lane_change_custom",
    script="scenarios/lane_change_custom.py",
    thesis_group="Lane Change Maneuvers",
    description="Custom lane-change/overtaking variant.",
),
```

Behavior thresholds can also be tuned in JSON:

```json
{
  "name": "lane_change_custom",
  "goal_x": 12.0,
  "lead_start_x": 10.0,
  "lead_initial_speed": 1.5,
  "lead_target_speed_kmh": 4.0,
  "behavior": {
    "time_gap": 1.0,
    "safety_distance": 2.0,
    "lane_change_speed_threshold": 2.77,
    "rrt_target_index_offset": 100
  }
}
```

### New Lead-Following Variant

For a new lead-following variant:

1. Create a small wrapper file in `scenarios/`.
2. Import `LeadVehicleScenarioConfig` and `run_scenario` from `scenarios.lead_vehicle_following`.
3. Define only the changed parameters.
4. Add the scenario to `scenarios/registry.py`.

Example:

```python
from scenarios.lead_vehicle_following import LeadVehicleScenarioConfig, run_scenario


CUSTOM_CONFIG = LeadVehicleScenarioConfig(
    ego_start=[1.5, 8.0, 0.0],
    ego_goal=[27.0, 32.0, 3.14159],
    lead_start=[9.0, 8.0, 0.0],
    lead_goal=[0.0, 32.0, 3.14159],
    ego_target_speed=15.0 / 3.6,
    lead_target_speed=10.0 / 3.6,
    lead_initial_speed=2.0,
)


def main():
    run_scenario(CUSTOM_CONFIG)
```

### New Standalone Scenario

Create a standalone scenario only when it needs different behavior logic. Reuse the shared helpers where possible:

- `simulation.parking_lot_map.build_parking_lot_obstacles`
- `simulation.path_setup.prepare_hybrid_path`
- `motion_planning.velocity_planning.calc_speed_profile`
- `simulation.plotting.plot_parking_lot_layout`
- `simulation.tracking.simulate_path_tracking`

## Running Scenarios

List scenarios:

```powershell
python scripts\run_scenario.py --list
```

Run a scenario headlessly:

```powershell
python scripts\run_scenario.py lane_change_3 --timeout 300
```

Run with live plotting:

```powershell
python scripts\run_scenario.py lane_change_3 --visual
```

Save a report-ready plot:

```powershell
python scripts\run_scenario.py lane_change_3 --timeout 300 --save-plot outputs\lane_change_3_velocity_profiles.png
```

Save the live visualization as a GIF:

```powershell
python scripts\run_scenario.py lane_change_3 --timeout 300 --save-gif outputs\lane_change_3_dynamic_obstacle_live.gif
```

Create a Markdown result table:

```powershell
python scripts\scenario_summary.py --scenarios static_obstacle_avoidance,lane_change_1,lane_change_2,lane_change_3 --timeout 300
```
