# Scenario Audit

Latest scenario smoke-check baseline.

Date: 2026-05-06

Command pattern:

```powershell
python scripts\run_scenario.py <scenario_name> --timeout 300 --json
```

## Results

| Scenario | Status | Goal | Sim Time | Notes |
| --- | --- | --- | ---: | --- |
| `static_environment` | ok | yes | 21.7 s | Single-vehicle Hybrid A* tracking. |
| `lead_vehicle_following` | ok | yes | 21.2 s | Shared lead-following runner. |
| `static_obstacle_avoidance` | ok | yes | 34.5 s | Stopped-obstacle lane change and return with RRT obstacle-footprint clearance check. |
| `lane_change_1` | ok | yes | 24.5 s | Close-obstacle lane change and return. |
| `lane_change_2` | ok | yes | 22.5 s | Delayed-obstacle lane change and return. |
| `lane_change_3` | ok | yes | 21.6 s | Extended-goal lane change and return. |

The lane-change scenarios log only the high-level simulation events:

- `simulation start`
- `lane change start`
- `lane change end`
- `return lane change start`
- `return lane change end`
- `simulation end`

## Verification Commands

Compile active code:

```powershell
python scripts\check_compile.py
```

Run fast tests:

```powershell
python -m unittest discover -s tests
```

Run selected scenario smoke tests through the unit-test harness:

```powershell
$env:RUN_SLOW_SCENARIO_SMOKE = "1"
$env:RUN_SCENARIO_SMOKE_NAMES = "lead_vehicle_following,lane_change_3"
python -m unittest discover -s tests
```

## Visualization

Run an interactive lane-change scenario:

```powershell
python scripts\run_scenario.py lane_change_3 --visual
```

Save a final figure:

```powershell
python scripts\run_scenario.py lane_change_3 --timeout 300 --save-plot outputs\lane_change_3_velocity_profiles.png
```

Save the live visualization as a GIF:

```powershell
python scripts\run_scenario.py lane_change_3 --timeout 300 --save-gif outputs\lane_change_3_dynamic_obstacle_live.gif
```

Generate a compact scenario table:

```powershell
python scripts\scenario_summary.py --scenarios static_obstacle_avoidance,lane_change_1,lane_change_2,lane_change_3 --timeout 300
```

The live plot displays:

- parking-lot boundaries as continuous lines,
- parking spaces as light outlined rectangles,
- optional lane markings as dashed yellow lines,
- ego trajectory during runtime,
- lead-vehicle trajectory during runtime,
- active RRT local path after it is found,
- current ego and lead vehicle frames.
