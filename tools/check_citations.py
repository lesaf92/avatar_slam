#!/usr/bin/env python3
"""Check citation hygiene of the manuscript (AGENTS.md §5).

* Every ``.bib`` entry must be preceded by a ``% status:`` line.
* Every ``\\cite{...}`` key must exist in the bib.
* ``--submission``: every *cited* entry must have status ``verified`` (not
  ``verified-web`` / ``UNVERIFIED``), and no ``\\todo`` / ``\\draftnote`` may remain.

Exit code 1 on any violation.

    python tools/check_citations.py            # draft mode (warnings for unverified)
    python tools/check_citations.py --submission
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PAPER = ROOT / "paper"

ENTRY_RE = re.compile(r"^@\w+\{([^,\s]+),", re.MULTILINE)
CITE_RE = re.compile(r"\\cite[pt]?\*?(?:\[[^\]]*\])?\{([^}]*)\}")


def bib_status(bib_text: str) -> dict[str, str]:
    """Map bib key -> status string (``missing`` if no status line precedes it)."""
    status: dict[str, str] = {}
    lines = bib_text.splitlines()
    for i, line in enumerate(lines):
        m = re.match(r"^@\w+\{([^,\s]+),", line)
        if not m:
            continue
        prev = lines[i - 1].strip() if i > 0 else ""
        s = re.match(r"^%\s*status:\s*(\S+)", prev)
        status[m.group(1)] = s.group(1) if s else "missing"
    return status


def cited_keys(tex_files: list[Path]) -> set[str]:
    keys: set[str] = set()
    for f in tex_files:
        text = re.sub(r"(?<!\\)%.*", "", f.read_text())  # drop comments
        for m in CITE_RE.finditer(text):
            keys.update(k.strip() for k in m.group(1).split(",") if k.strip())
    return keys


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--submission", action="store_true")
    args = ap.parse_args()

    status = bib_status((PAPER / "references.bib").read_text())
    tex = [PAPER / "main.tex", *sorted((PAPER / "sections").glob("*.tex"))]
    cited = cited_keys(tex)
    errors: list[str] = []
    warnings: list[str] = []

    for key, st in status.items():
        if st == "missing":
            errors.append(f"bib entry {key} has no '% status:' line")
    for key in sorted(cited - set(status)):
        errors.append(f"cited key {key} not found in references.bib")
    for key in sorted(cited & set(status)):
        st = status[key]
        if st != "verified":
            (errors if args.submission else warnings).append(f"cited {key} has status {st}")
    if args.submission:
        for f in tex:
            text = re.sub(r"(?<!\\)%.*", "", f.read_text())
            n = len(re.findall(r"\\(todo|draftnote)\{", text))
            if n and f.name != "main.tex":
                errors.append(f"{f.relative_to(ROOT)} still has {n} \\todo/\\draftnote")

    for w in warnings:
        print(f"warning: {w}")
    for e in errors:
        print(f"error: {e}")
    counts = {s: sum(1 for k in cited if status.get(k) == s) for s in set(status.values())}
    print(f"{len(cited)} cited keys; status of cited entries: {counts}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
