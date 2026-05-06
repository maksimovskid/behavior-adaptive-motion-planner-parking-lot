"""Compile-check repository Python files."""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def iter_python_files() -> list[Path]:
    """Return active repository Python files that should compile."""
    files = []
    for path in REPO_ROOT.rglob("*.py"):
        relative = path.relative_to(REPO_ROOT)
        if "__pycache__" in relative.parts:
            continue
        files.append(path)
    return sorted(files)


def check_compile() -> tuple[list[Path], list[tuple[Path, Exception]]]:
    """Compile every active Python file and collect failures."""
    compiled: list[Path] = []
    failures: list[tuple[Path, Exception]] = []
    for path in iter_python_files():
        try:
            source = path.read_text(encoding="utf-8", errors="replace")
            compile(source, str(path), "exec")
            compiled.append(path)
        except Exception as exc:
            failures.append((path, exc))
    return compiled, failures


def main() -> int:
    """CLI entry point for the compile check."""
    compiled, failures = check_compile()
    print(f"compiled: {len(compiled)}")

    if failures:
        print("failures:", file=sys.stderr)
        for path, exc in failures:
            print(f"- {path.relative_to(REPO_ROOT)}: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
