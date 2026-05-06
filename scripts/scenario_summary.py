"""Run selected scenarios and print a compact result table."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scenarios.registry import SCENARIOS, get_scenario
from simulation.scenario_runner import run_scenario


def _parse_names(value: str | None) -> list[str]:
    """Parse a comma-separated scenario list or return all registered names."""
    if not value:
        return list(SCENARIOS)
    return [name.strip() for name in value.split(",") if name.strip()]


def _markdown_table(results: list[dict[str, object]]) -> str:
    """Format scenario runner results as a Markdown table."""
    lines = [
        "| Scenario | Status | Goal | Sim Time | Events |",
        "| --- | --- | --- | ---: | --- |",
    ]
    for result in results:
        events = "; ".join(
            line
            for line in str(result["stdout_tail"]).splitlines()
            if "lane change" in line
        )
        sim_time = result["total_time_seconds"]
        sim_time_text = "" if sim_time is None else f"{sim_time} s"
        lines.append(
            f"| `{result['scenario']}` | {result['status']} | {result['goal_reached']} | "
            f"{sim_time_text} | {events} |"
        )
    return "\n".join(lines)


def _write_csv(path: Path, results: list[dict[str, object]]) -> None:
    """Write selected runner fields to a CSV file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "scenario",
                "status",
                "goal_reached",
                "total_time_seconds",
                "elapsed_seconds",
                "summary",
            ],
        )
        writer.writeheader()
        for result in results:
            writer.writerow({field: result[field] for field in writer.fieldnames})


def main(argv: list[str] | None = None) -> int:
    """Run selected scenarios and print/write a compact summary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenarios",
        help="Comma-separated scenario names. Defaults to all registered scenarios.",
    )
    parser.add_argument("--timeout", type=int, default=300)
    parser.add_argument("--csv", help="Optional CSV output path.")
    args = parser.parse_args(argv)

    results = []
    for name in _parse_names(args.scenarios):
        result = run_scenario(get_scenario(name), timeout_seconds=args.timeout)
        results.append(result)

    print(_markdown_table(results))
    if args.csv:
        _write_csv(Path(args.csv), results)

    return 0 if all(result["status"] == "ok" and result["goal_reached"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
