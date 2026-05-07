# Outputs

This folder is for local generated artifacts from scenario runs.

Use it when you run experiments, regenerate plots, export GIFs, or create result tables. Most generated files in this folder are local artifacts; only this README is tracked by default.

The repository also contains `docs/figures/`. That folder has a different purpose: it stores the example media shown in the project README. In other words:

- `docs/figures/`: stable media used by documentation.
- `outputs/`: temporary or local media/results created while running scenarios.

If you create a new figure or GIF for the README or documentation, first generate it in `outputs/`, inspect it, then copy or regenerate the final version into `docs/figures/` with a descriptive filename.

## Examples

Save a static figure:

```powershell
python scripts\run_scenario.py lane_change_3 --timeout 300 --save-plot outputs\lane_change_3_velocity_profiles.png
```

Save a live visualization GIF:

```powershell
python scripts\run_scenario.py lane_change_3 --timeout 300 --save-gif outputs\lane_change_3_dynamic_obstacle_live.gif
```

Save a scenario summary table:

```powershell
python scripts\scenario_summary.py --scenarios lane_change_1,lane_change_2,lane_change_3 --timeout 300 --csv outputs\scenario_summary.csv
```

Generated images, GIFs, videos, and CSV files in this folder are ignored by default.
