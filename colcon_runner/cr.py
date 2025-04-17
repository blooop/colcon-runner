#!/usr/bin/env python3
"""Colcon Runner (cr): a concise CLI wrapper around common *colcon* tasks.

Usage pattern (see README for full details):
    cr VERB[SPEC] [PKG] [--dry-run]

Examples:
    cr ba              # build *all* packages
    cr bo my_pkg       # build *only* my_pkg
    cr cu my_pkg       # clean *up‑to* my_pkg (and its deps)
    cr cabu            # clean all, then build up‑to default pkg

The single‑letter verbs map to *colcon* sub‑commands, while specifiers
convert to the appropriate package‑selection flags.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

# ---------------------------------------------------------------------------
# Configuration helpers
# ---------------------------------------------------------------------------

# Allow the config file location to be overridden in tests via $CR_CONFIG.
CONFIG_PATH = Path(os.environ.get("CR_CONFIG", Path.home() / ".cr_config.json"))


def _load_default_package() -> str | None:
    """Return the stored default package name, or *None* if unset/invalid."""
    if not CONFIG_PATH.exists():
        return None
    try:
        with CONFIG_PATH.open(encoding="utf-8") as fp:
            return json.load(fp).get("default_package")  # type: ignore[arg-type]
    except (json.JSONDecodeError, OSError):
        return None


def _save_default_package(pkg: str) -> None:
    """Persist *pkg* as the default package."""
    CONFIG_PATH.write_text(json.dumps({"default_package": pkg}), encoding="utf-8")


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

action_map: dict[str, str] = {
    "b": "build",
    "t": "test",
    "c": "clean",
}

specifier_flags: dict[str, List[str] | None] = {
    "a": None,  # all packages – no extra flags
    "o": ["--packages-select"],
    "u": ["--packages-up-to"],
}


class ParseError(RuntimeError):
    """Raised when the verb string is malformed."""


VerbSpec = Tuple[str, str | None]


def _parse_verbs(verb_str: str) -> List[VerbSpec]:
    """Convert *verb_str* into a list of ``(verb, spec)`` tuples.

    The special verb ``s`` (set default) has no specifier and must appear
    alone.
    """
    if verb_str == "s":
        return [("s", None)]

    if len(verb_str) % 2:  # must be pairs of two characters
        raise ParseError(f"Odd number of characters in verb string '{verb_str}'.")

    parsed: List[VerbSpec] = []
    for v, s in zip(verb_str[::2], verb_str[1::2]):
        if v not in action_map:
            raise ParseError(f"Unknown verb '{v}' in '{verb_str}'.")
        if s not in specifier_flags:
            raise ParseError(f"Unknown specifier '{s}' in '{verb_str}'.")
        parsed.append((v, s))
    return parsed


# ---------------------------------------------------------------------------
# Command builder
# ---------------------------------------------------------------------------


def _build_colcon_cmd(verb: str, spec: str, pkg: str | None) -> List[str]:
    """Return the concrete *colcon* command for a verb/spec pair."""
    base = ["colcon", action_map[verb]]

    flags = specifier_flags[spec]
    if flags is None:  # 'a' – no package‑selection flag
        return base

    if not pkg:
        raise ParseError("Package name required for 'only'/'up‑to' specifiers.")

    return base + flags + [pkg]


# ---------------------------------------------------------------------------
# Public CLI entry‑point
# ---------------------------------------------------------------------------


def _build_all_cmds(parsed: Sequence[VerbSpec], pkg: str | None) -> List[List[str]]:
    cmds: List[List[str]] = []
    for verb, spec in parsed:
        cmds.append(_build_colcon_cmd(verb, spec, pkg))
    return cmds


def main(argv: Sequence[str] | None = None) -> int:  # noqa: C901 – top‑level function
    argv = list(argv or sys.argv[1:])

    # Default action: build all
    if not argv:
        argv = ["ba"]

    parser = argparse.ArgumentParser(
        prog="cr",
        description="Colcon Runner – concise wrapper around common *colcon* tasks.",
    )
    parser.add_argument("verbstr", help="Verb/specifier string or 's' to set default package")
    parser.add_argument("pkg", nargs="?", help="Package name (optional – can fall back to default)")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without executing")

    args = parser.parse_args(argv)
    verbstr: str = args.verbstr

    # ------------------------------------------------------------------
    # Handle the special 's' verb (set default package) early & return.
    # ------------------------------------------------------------------
    if verbstr == "s":
        if not args.pkg:
            parser.error("Package name is required with the 's' verb.")
        _save_default_package(args.pkg)
        print(f"Default package set to '{args.pkg}'.")
        return 0

    # ------------------------------------------------------------------
    # Parse general verb strings
    # ------------------------------------------------------------------
    try:
        parsed = _parse_verbs(verbstr)
    except ParseError as exc:
        parser.error(str(exc))

    pkg_arg = args.pkg or _load_default_package()

    # Verify that we have a package name if *any* command needs it.
    if any(spec in ("o", "u") for _verb, spec in parsed) and not pkg_arg:
        parser.error("This command requires a package name (none provided and no default set).")

    # Build list of colcon commands to execute
    try:
        commands = _build_all_cmds(parsed, pkg_arg)
    except ParseError as exc:
        parser.error(str(exc))

    # Execute (or echo) them sequentially
    for cmd in commands:
        if args.dry_run:
            print("$", *cmd)
            continue

        print("\n»", *cmd)
        result = subprocess.run(cmd, check=False)
        if result.returncode:
            return result.returncode

    return 0


if __name__ == "__main__":  # pragma: no cover – direct invocation
    raise SystemExit(main())
