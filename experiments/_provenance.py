"""Commit label for experiment outputs (AGENTS.md §5).

``git describe --dirty`` would flag a tree as dirty as soon as an earlier step
of ``make paper-data`` rewrote a tracked CSV. Output directories are therefore
excluded: the label says ``-dirty`` only if code, configs or docs differ from
the commit.
"""

from __future__ import annotations

import subprocess

OUTPUT_DIRS = ("paper/data", "viz/data")


def commit() -> str:
    """Short hash of HEAD, plus ``-dirty`` if tracked non-output files changed."""
    try:
        rev = subprocess.run(
            ["git", "rev-parse", "--short=7", "HEAD"],
            capture_output=True, text=True, check=True,
        ).stdout.strip()  # fmt: skip
        exclude = [f":(top,exclude){d}" for d in OUTPUT_DIRS]
        status = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no", "--", ":/", *exclude],
            capture_output=True, text=True, check=True,
        ).stdout.strip()  # fmt: skip
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    return rev + ("-dirty" if status else "")
