"""Run thesis scenario scripts in a controlled, non-interactive way."""

from __future__ import annotations

import argparse
import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    from scenarios.registry import REPO_ROOT, SCENARIOS, Scenario, get_scenario
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from scenarios.registry import REPO_ROOT, SCENARIOS, Scenario, get_scenario


CHILD_RUNNER = r"""
import runpy
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.ioff()
plt.rcParams["figure.max_open_warning"] = 0
plt.pause = lambda *args, **kwargs: None
plt.show = lambda *args, **kwargs: None

save_plot_path = SAVE_PLOT_PATH
sys.argv = [SCRIPT_PATH]
runpy.run_path(SCRIPT_PATH, run_name="__main__")
if save_plot_path is not None:
    output_path = Path(save_plot_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=180, bbox_inches="tight")
"""


CHILD_VISUAL_RUNNER = r"""
import runpy
import sys

sys.argv = [SCRIPT_PATH]
namespace = runpy.run_path(SCRIPT_PATH, run_name="__scenario_visual__")
namespace["show_animation"] = True
namespace["main"].__globals__["show_animation"] = True
namespace["main"]()
"""


CHILD_GIF_RUNNER = r"""
import io
import runpy
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

plt.ioff()
plt.rcParams["figure.max_open_warning"] = 0
plt.show = lambda *args, **kwargs: None

gif_path = Path(GIF_PATH)
frame_stride = FRAME_STRIDE
frame_duration_ms = FRAME_DURATION_MS
max_frames = MAX_FRAMES
frames = []
pause_count = 0


def capture_current_figure():
    if len(frames) >= max_frames:
        return
    figure = plt.gcf()
    figure.canvas.draw()
    buffer = io.BytesIO()
    figure.savefig(buffer, format="png", dpi=GIF_DPI, bbox_inches="tight")
    buffer.seek(0)
    frames.append(Image.open(buffer).convert("RGB").copy())


def record_pause(*args, **kwargs):
    global pause_count
    pause_count += 1
    if pause_count % frame_stride == 0:
        capture_current_figure()


plt.pause = record_pause
sys.argv = [SCRIPT_PATH]
namespace = runpy.run_path(SCRIPT_PATH, run_name="__scenario_gif__")
namespace["show_animation"] = True
namespace["main"].__globals__["show_animation"] = True
namespace["main"]()
if not frames:
    capture_current_figure()
gif_path.parent.mkdir(parents=True, exist_ok=True)
frames[0].save(
    gif_path,
    save_all=True,
    append_images=frames[1:],
    duration=frame_duration_ms,
    loop=0,
    optimize=False,
)
"""


def list_scenarios() -> list[dict[str, str]]:
    """Return registry metadata for CLI listing and JSON output."""
    return [
        {
            "name": scenario.name,
            "script": scenario.script,
            "thesis_group": scenario.thesis_group,
            "description": scenario.description,
        }
        for scenario in SCENARIOS.values()
    ]


def run_scenario(
    scenario: Scenario,
    timeout_seconds: int | None = None,
    stop_after_goal: bool = True,
    save_plot_path: str | None = None,
    save_gif_path: str | None = None,
    gif_frame_stride: int = 3,
    gif_frame_duration_ms: int = 80,
    gif_max_frames: int = 180,
    gif_dpi: int = 90,
) -> dict[str, object]:
    """Run one registered scenario in a subprocess and summarize the result."""
    if not scenario.script_path.exists():
        return {
            "scenario": scenario.name,
            "script": scenario.script,
            "status": "missing",
            "returncode": None,
            "goal_reached": False,
            "elapsed_seconds": 0.0,
            "summary": f"Script does not exist: {scenario.script_path}",
            "stdout_tail": "",
            "stderr_tail": "",
        }

    timeout = timeout_seconds or scenario.timeout_seconds
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    env["PYTHONWARNINGS"] = env.get("PYTHONWARNINGS", "default")

    if save_gif_path is not None:
        code = CHILD_GIF_RUNNER.replace("SCRIPT_PATH", repr(str(scenario.script_path)))
        code = code.replace("GIF_PATH", repr(save_gif_path))
        code = code.replace("FRAME_STRIDE", repr(gif_frame_stride))
        code = code.replace("FRAME_DURATION_MS", repr(gif_frame_duration_ms))
        code = code.replace("MAX_FRAMES", repr(gif_max_frames))
        code = code.replace("GIF_DPI", repr(gif_dpi))
    else:
        code = CHILD_RUNNER.replace("SCRIPT_PATH", repr(str(scenario.script_path)))
        code = code.replace("SAVE_PLOT_PATH", repr(save_plot_path))
    started = time.monotonic()
    status = "ok"

    stdout, stderr, returncode, stopped_after_goal = _run_child(
        code=code,
        timeout=timeout,
        env=env,
        stop_after_goal=stop_after_goal,
        goal_message=scenario.expected_goal_message,
    )
    if returncode is None:
        status = "timeout"

    elapsed = round(time.monotonic() - started, 2)
    if returncode not in (0, None):
        status = "failed"

    goal_reached = scenario.expected_goal_message in stdout
    total_time = _extract_last_float(r"total time:\s+([0-9.]+)", stdout)
    path_length = _extract_last_float(r"length of path:\s+([0-9.]+)", stdout)

    summary_bits = []
    if goal_reached:
        summary_bits.append("goal reached")
    if total_time is not None:
        summary_bits.append(f"sim time {total_time}s")
    if path_length is not None:
        summary_bits.append(f"path {path_length}m")
    if not summary_bits:
        summary_bits.append("no goal marker found")

    return {
        "scenario": scenario.name,
        "script": scenario.script,
        "status": status,
        "returncode": returncode,
        "goal_reached": goal_reached,
        "elapsed_seconds": elapsed,
        "stopped_after_goal": stopped_after_goal,
        "total_time_seconds": total_time,
        "path_length_meters": path_length,
        "summary": ", ".join(summary_bits),
        "stdout_tail": _tail(stdout),
        "stderr_tail": _tail(stderr),
        "saved_plot": save_plot_path,
        "saved_gif": save_gif_path,
    }


def run_scenario_visual(scenario: Scenario) -> int:
    """Run a scenario in normal Matplotlib mode so plot windows are visible."""

    if not scenario.script_path.exists():
        print(f"Script does not exist: {scenario.script_path}", file=sys.stderr)
        return 1

    env = os.environ.copy()
    env.pop("MPLBACKEND", None)
    code = CHILD_VISUAL_RUNNER.replace("SCRIPT_PATH", repr(str(scenario.script_path)))
    return subprocess.call([sys.executable, "-c", code], cwd=str(REPO_ROOT), env=env)


def _run_child(
    code: str,
    timeout: int,
    env: dict[str, str],
    stop_after_goal: bool,
    goal_message: str,
) -> tuple[str, str, int | None, bool]:
    """Execute child Python code while capturing stdout/stderr and enforcing timeout."""
    process = subprocess.Popen(
        [sys.executable, "-c", code],
        cwd=str(REPO_ROOT),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=1,
    )

    output_queue: queue.Queue[tuple[str, str | None]] = queue.Queue()
    stdout_lines: list[str] = []
    stderr_lines: list[str] = []

    def read_stream(name: str, stream) -> None:
        """Forward one child stream into the shared output queue."""
        try:
            for line in iter(stream.readline, ""):
                output_queue.put((name, line))
        finally:
            output_queue.put((name, None))

    threads = [
        threading.Thread(target=read_stream, args=("stdout", process.stdout), daemon=True),
        threading.Thread(target=read_stream, args=("stderr", process.stderr), daemon=True),
    ]
    for thread in threads:
        thread.start()

    started = time.monotonic()
    streams_done = set()
    stopped_after_goal = False

    while process.poll() is None or len(streams_done) < 2:
        if time.monotonic() - started > timeout:
            process.kill()
            break

        try:
            stream_name, line = output_queue.get(timeout=0.2)
        except queue.Empty:
            continue

        if line is None:
            streams_done.add(stream_name)
            continue

        if stream_name == "stdout":
            stdout_lines.append(line)
            stdout_text = "".join(stdout_lines)
            if stop_after_goal and goal_message in stdout_text and "total time:" in stdout_text:
                stopped_after_goal = True
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                break
        else:
            stderr_lines.append(line)

    for thread in threads:
        thread.join(timeout=1)

    returncode = process.poll()
    if returncode is None:
        process.kill()

    while not output_queue.empty():
        stream_name, line = output_queue.get_nowait()
        if line is None:
            continue
        if stream_name == "stdout":
            stdout_lines.append(line)
        else:
            stderr_lines.append(line)

    if stopped_after_goal:
        returncode = 0

    return "".join(stdout_lines), "".join(stderr_lines), returncode, stopped_after_goal


def _extract_last_float(pattern: str, text: str) -> float | None:
    """Extract the last floating-point value matching a regex group."""
    matches = re.findall(pattern, text)
    if not matches:
        return None
    return float(matches[-1])


def _tail(text: str, max_lines: int = 40) -> str:
    """Return the last few lines of captured process output."""
    lines = text.splitlines()
    return "\n".join(lines[-max_lines:])


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for listing, running, plotting, and GIF-exporting scenarios."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", nargs="?", help="Scenario name to run.")
    parser.add_argument("--list", action="store_true", help="List registered scenarios.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    parser.add_argument("--visual", action="store_true", help="Open the scenario plot window.")
    parser.add_argument("--save-plot", help="Save the final Matplotlib figure to this image path.")
    parser.add_argument("--save-gif", help="Save the live visualization frames to this GIF path.")
    parser.add_argument("--gif-frame-stride", type=int, default=3, help="Record every Nth live frame.")
    parser.add_argument("--gif-frame-duration", type=int, default=80, help="GIF frame duration in milliseconds.")
    parser.add_argument("--gif-max-frames", type=int, default=180, help="Maximum number of GIF frames to store.")
    parser.add_argument("--gif-dpi", type=int, default=90, help="DPI used for captured GIF frames.")
    parser.add_argument("--timeout", type=int, help="Override scenario timeout in seconds.")
    parser.add_argument("--show-output", action="store_true", help="Print captured stdout/stderr tails.")
    parser.add_argument(
        "--no-stop-after-goal",
        action="store_true",
        help="Let the original script finish naturally instead of stopping after the goal summary.",
    )
    args = parser.parse_args(argv)

    if args.list:
        scenarios = list_scenarios()
        if args.json:
            print(json.dumps(scenarios, indent=2))
        else:
            for scenario in scenarios:
                print(f"{scenario['name']}: {scenario['script']} ({scenario['thesis_group']})")
        return 0

    if not args.scenario:
        parser.error("provide a scenario name or use --list")

    try:
        scenario = get_scenario(args.scenario)
    except KeyError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if args.visual:
        return run_scenario_visual(scenario)

    result = run_scenario(
        scenario,
        timeout_seconds=args.timeout,
        stop_after_goal=not args.no_stop_after_goal and not args.save_plot and not args.save_gif,
        save_plot_path=args.save_plot,
        save_gif_path=args.save_gif,
        gif_frame_stride=args.gif_frame_stride,
        gif_frame_duration_ms=args.gif_frame_duration,
        gif_max_frames=args.gif_max_frames,
        gif_dpi=args.gif_dpi,
    )
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(f"{result['scenario']}: {result['status']} - {result['summary']}")
        print(f"script: {result['script']}")
        print(f"elapsed: {result['elapsed_seconds']}s")
        if args.show_output:
            if result["stdout_tail"]:
                print("\nstdout tail:")
                print(result["stdout_tail"])
            if result["stderr_tail"]:
                print("\nstderr tail:")
                print(result["stderr_tail"])

    return 0 if result["status"] == "ok" and result["goal_reached"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
