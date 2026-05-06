# Repository Structure

The active code is organized by responsibility: planning, vehicle model, simulation infrastructure, and runnable scenarios.

## Top-Level Layout

```text
motion_planning/
vehicle/
simulation/
scenarios/
scripts/
tests/
docs/
```

## Motion Planning

`motion_planning/global_planning/`

- `a_star.py`: grid-based A* planning support.
- `hybrid_a_star.py`: Hybrid A* global path planner.
- `reeds_shepp_path_planning.py`: Reeds-Shepp path primitives.

`motion_planning/local_planning/`

- `behaviour_and_local_planner.py`: behavior state machine and RRT lane-change path generation.
- `rrt.py`, `rrt_1.py`: RRT planning implementations.
- `rrt_reeds_shepp.py`: RRT planner using Reeds-Shepp connections.

`motion_planning/path_interpolation/`

- `cubic_spline_hybrid_a_star.py`: spline utilities used to interpolate Hybrid A* paths.
- `cubic_spline_planner.py`: 2D cubic spline path interpolation and curvature sampling.

`motion_planning/velocity_planning.py`

- Shared speed-profile helpers for static, lead-vehicle, and lane-change scenarios.

## Vehicle

`vehicle/`

- `car.py`: vehicle geometry, drawing, collision checks, and steering limits.
- `controllers.py`: longitudinal speed control and rear-wheel feedback lateral control.
- `kinematics.py`: vehicle state, kinematic update, and constant-acceleration equations.

## Simulation

`simulation/`

- `parking_lot_map.py`: parking-lot obstacle and RRT obstacle generation.
- `path_setup.py`: shared Hybrid A* path preparation.
- `plotting.py`: semantic parking-lot layout, vehicle, trajectory, and speed-profile plotting helpers.
- `tracking.py`: reusable closed-loop path-tracking simulation for simple scenarios.
- `scenario_runner.py`: non-interactive scenario execution used by the CLI.

## Scenarios

`scenarios/`

- `static_environment.py`: single-vehicle Hybrid A* tracking scenario.
- `lead_vehicle_following.py`: shared two-vehicle lead-following simulation.
- `static_obstacle_avoidance.py`: stopped-obstacle avoidance configuration.
- `lane_change.py`: shared lane-change simulation engine.
- `lane_change_close_obstacle.py`: close-obstacle lane-change configuration.
- `lane_change_delayed_obstacle.py`: delayed-obstacle lane-change configuration.
- `lane_change_extended_goal.py`: longer-goal lane-change configuration.
- `registry.py`: stable scenario names used by the runner.

The lead-following and lane-change variants share implementations and differ only by configuration values such as goal position, lead-vehicle start, lead-vehicle speed, and behavior thresholds.

## Scripts

`scripts/`

- `run_scenario.py`: list and run registered scenarios.
- `check_compile.py`: compile-check repository Python files.
- `scenario_summary.py`: run selected scenarios and print a Markdown/CSV result table.
- `scenarios.py`: compatibility wrapper for scenario access.

## Tests

`tests/`

- Fast tests cover the scenario registry, CLI runner, and compile-check behavior.
- Slow scenario smoke tests are opt-in through environment variables.
