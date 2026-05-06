# Development Workflow

This repository is a thesis simulation project. Prefer small, verified changes.

## Local Checks

Run before committing:

```powershell
python scripts\verify.py
```

For a targeted scenario check:

```powershell
python scripts\run_scenario.py lane_change_3 --timeout 300 --json
```

## Adding Scenarios

Prefer adding small configuration wrappers over copying full scenario scripts. See `docs/scenario_inventory.md` for examples.

## Generated Outputs

Use `outputs/` for generated figures and CSV result tables. Generated files are ignored by git except for `outputs/README.md`.
