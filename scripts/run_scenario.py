"""CLI wrapper for the structured scenario runner."""

from __future__ import annotations

from pathlib import Path
import sys


if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.scenario_runner import main


if __name__ == "__main__":
    raise SystemExit(main())

