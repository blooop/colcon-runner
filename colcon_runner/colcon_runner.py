#!/usr/bin/env python3
"""
CR(1)                         User Commands                        CR(1)

NAME
    cr - Colcon Runner: concise CLI for common colcon tasks.

SYNOPSIS
    cr VERB [PKG] [OPTIONS]

DESCRIPTION
    A minimal wrapper around colcon providing short, mnemonic commands
    for build, test, clean, and package selection operations.

STATE
    s       set a default package for subsequent commands.

VERBS
    u       underlay (when used as first verb, operates on underlay workspace)
    b       build packages.
    t       Test packages.
    c       clean packages.

SPECIFIER
    o       only (--packages-select)
    u       upto (when used after verb: --packages-up-to)
    a       all (default if omitted)

If no specifier is provided after a verb, it defaults to "a" (all). You can chain as many verb-specifier pairs as you want. You can set a default package to use for all subsequent commands, or you can specify a package in the command itself.

USAGE EXAMPLES

  Basic Commands:
    cr b
        Build all packages. (shorthand, specifier defaults to "a")

    cr ba
        Build all packages. (explicit)

    cr bo pkg_1
        Build only 'pkg_1'.

    cr bu pkg_1
        Build upto 'pkg_1' and its dependencies.

    cr t
        Test all packages. (shorthand)

    cr ta
        Test all packages. (explicit)

    cr to pkg_1
        Test only 'pkg_1'.

    cr tu pkg_1
        Test upto 'pkg_1' and its dependencies.

    cr c
        Clean workspace. (shorthand)

    cr ca
        Clean workspace (build/, install/, log/, and test_result/ directories)

    cr co pkg_1
        Clean only 'pkg_1'.

    cr cu pkg_1
        Clean upto 'pkg_1'.

  Compound Commands:
    cr s pkg1
        Set 'pkg_1' as the default package for subsequent commands.

    cr bt
        Build all and test all. (shorthand)

    cr cbt
        Clean all, build all, and test all. (shorthand)

    cr cabu
        Clean all and build up to 'pkg1'.

    cr boto
        build only 'pkg1' package, then test only 'pkg1'.

    cr cabuto
        Clean all, build up to 'pkg1', and test only 'pkg1'.

  Underlay Commands:
    cr u b
        Build all packages in the underlay workspace.

    cr u bo pkg_1
        Build only 'pkg_1' in the underlay.

    cr u bt
        Build and test all packages in the underlay.

    cr --underlay /path/to/underlay b
        Build all packages in the specified underlay path.


NOTES
    - The 's' verb sets a default package name stored in a configuration file.
    - Subsequent commands that require a package argument will use the default if none is provided.
    - Compound verbs can be chained together for streamlined operations.
    - Underlay path can be set in colcon-defaults.yaml with 'colcon-runner-underlay: "/path"' or via --underlay flag.
    - When 'u' is the first verb, it operates on the underlay. When 'u' follows a verb (bu, tu, cu), it means 'upto'.

SEE ALSO
    colcon(1), colcon-clean(1)
"""

import sys
import os
import subprocess
import yaml
from typing import Optional, List, Tuple
from pathlib import Path

PKG_FILE: str = os.path.expanduser("~/.colcon_shortcuts_pkg")


class ParseError(Exception):
    pass


def _parse_verbs(cmds: str) -> Tuple[bool, List[Tuple[str, Optional[str]]]]:
    """Parse a string like 'boto' into [(verb, spec), ...].
    Returns (is_underlay, parsed_verbs)."""
    is_underlay = False
    result = []
    i = 0

    # Check if first character is 'u' for underlay mode
    if len(cmds) > 0 and cmds[0] == "u":
        # If it's just "u" or next char is a verb, treat as underlay
        if len(cmds) == 1 or cmds[1] in ("s", "b", "t", "c"):
            is_underlay = True
            i = 1  # Skip the 'u'

    while i < len(cmds):
        if cmds[i] in ("s", "b", "t", "c"):
            verb = cmds[i]
            if verb == "s":
                result.append((verb, None))
                i += 1
                continue
            # If no specifier provided or invalid specifier, default to "a"
            if i + 1 >= len(cmds) or cmds[i + 1] not in ("o", "u", "a"):
                result.append((verb, "a"))
                i += 1
            else:
                result.append((verb, cmds[i + 1]))
                i += 2
        else:
            raise ParseError(f"unknown command letter '{cmds[i]}'")
    return is_underlay, result


def _build_colcon_cmd(verb, spec, pkg):
    if verb == "b":
        args = ["build"]
    elif verb == "t":
        args = ["test"]
    elif verb == "c":
        args = [
            "clean",
            "workspace",
            "--yes",
            "--base-select",
            "build",
            "install",
            "log",
            "test_result",
        ]
    else:
        raise ParseError(f"unsupported verb '{verb}'")
    if spec == "o":
        if not pkg:
            raise ParseError(f"{verb} 'only' requires a package name")
        args.extend(["--packages-select", pkg])
    elif spec == "u":
        if not pkg:
            raise ParseError(f"{verb} 'upto' requires a package name")
        args.extend(["--packages-up-to", pkg])
    elif spec == "a":
        pass
    else:
        raise ParseError(f"unknown specifier '{spec}'")
    return args


def load_default_pkg() -> Optional[str]:
    if os.path.isfile(PKG_FILE):
        with open(PKG_FILE, "r", encoding="utf-8") as f:
            return f.read().strip()
    return None


def save_default_pkg(pkg: str) -> None:
    with open(PKG_FILE, "w", encoding="utf-8") as f:
        f.write(pkg)
    print(f"Default package set to '{pkg}'")


def error(msg: str) -> None:
    print(f"Error: {msg}", file=sys.stderr)
    sys.exit(1)


def get_pkg(override: Optional[str]) -> str:
    if override:
        return override
    if default := load_default_pkg():
        return default
    error("no package specified and no default set")
    return None


def find_colcon_defaults() -> Optional[Path]:
    """Find colcon-defaults.yaml by searching upward from current directory."""
    current = Path.cwd()
    for parent in [current] + list(current.parents):
        defaults_path = parent / "colcon-defaults.yaml"
        if defaults_path.exists():
            return defaults_path
    return None


def load_underlay_path_from_config() -> Optional[str]:
    """Load underlay path from colcon-defaults.yaml."""
    defaults_path = find_colcon_defaults()
    if not defaults_path:
        return None

    try:
        with open(defaults_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
            if config and "colcon-runner-underlay" in config:
                return config["colcon-runner-underlay"]
    except Exception:
        pass
    return None


def run_colcon(args: List[str], extra_opts: List[str], underlay_path: Optional[str] = None) -> None:
    # Defensive: ensure all args are strings and not user-controlled shell input
    import shlex

    safe_args = [str(a) for a in args]
    safe_extra_opts = [str(a) for a in extra_opts]
    cmd = ["colcon"] + safe_args + safe_extra_opts

    # Change to underlay directory if specified
    original_cwd = None
    if underlay_path:
        original_cwd = os.getcwd()
        underlay_path = os.path.expanduser(underlay_path)
        if not os.path.isdir(underlay_path):
            error(f"underlay path does not exist: {underlay_path}")
        os.chdir(underlay_path)
        print(f"[underlay: {underlay_path}]")

    print("+ " + " ".join(shlex.quote(a) for a in cmd))
    # Use subprocess.run with shell=False for safety
    ret = subprocess.run(cmd, check=False).returncode

    # Restore original directory
    if original_cwd:
        os.chdir(original_cwd)

    if ret != 0:
        sys.exit(ret)


def main(argv=None) -> None:
    if argv is None:
        argv = sys.argv[1:]
    if len(argv) < 1:
        print(
            "No arguments provided. Running 'colcon build' by default.\nUse '--help' for more options."
        )
        run_colcon(["build"], [])
        sys.exit(0)

    # Add --help and -h support
    if argv[0] in ("--help", "-h"):
        print(__doc__)
        sys.exit(0)

    cmds: str = argv[0]
    rest: List[str] = argv[1:]

    # extract override pkg (first non-dash arg) and --underlay flag
    override_pkg: Optional[str] = None
    underlay_override: Optional[str] = None
    extra_opts: List[str] = []
    i = 0
    while i < len(rest):
        arg = rest[i]
        if arg == "--underlay":
            if i + 1 >= len(rest):
                error("--underlay requires a path argument")
            underlay_override = rest[i + 1]
            i += 2
        elif not arg.startswith("-") and override_pkg is None:
            override_pkg = arg
            i += 1
        else:
            extra_opts.append(arg)
            i += 1

    is_underlay, parsed_verbs = _parse_verbs(cmds)

    # Determine underlay path
    underlay_path: Optional[str] = None
    if is_underlay or underlay_override:
        underlay_path = underlay_override or load_underlay_path_from_config()
        if not underlay_path:
            error("underlay mode requested but no underlay path found (use --underlay or set colcon-runner-underlay in colcon-defaults.yaml)")

    # execute each segment
    for verb, spec in parsed_verbs:
        if verb == "s":
            # set default package
            if not override_pkg:
                error("'s' requires a package name")
            save_default_pkg(override_pkg)
            # do not run colcon for 's'
            continue

        # determine pkg if needed
        need_pkg: bool = spec in ("o", "u")
        pkg: Optional[str] = get_pkg(override_pkg) if need_pkg else None
        args: List[str] = _build_colcon_cmd(verb, spec, pkg)

        # Support --dry-run for tests
        if "--dry-run" in extra_opts:
            print("+ " + " ".join(args + extra_opts))
            continue

        run_colcon(args, extra_opts, underlay_path)


if __name__ == "__main__":
    main()
