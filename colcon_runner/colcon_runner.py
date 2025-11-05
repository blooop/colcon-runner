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
    b       build packages.
    t       Test packages.
    c       clean packages.
    i       install dependencies using rosdep.

SPECIFIER
    o       only (--packages-select)
    u       upto (--packages-up-to)
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

    cr i
        Install all dependencies using rosdep. (shorthand)

    cr ia
        Install all dependencies using rosdep. (explicit)

    cr io pkg_1
        Install dependencies only for 'pkg_1'.

    cr iu pkg_1
        Install dependencies for 'pkg_1' and its dependencies.

  Compound Commands:
    cr s pkg1
        Set 'pkg_1' as the default package for subsequent commands.

    cr bt
        Build all and test all. (shorthand)

    cr cbt
        Clean all, build all, and test all. (shorthand)

    cr ib
        Install all dependencies and build all.

    cr iobo
        Install dependencies for 'pkg1' only, then build only 'pkg1'.

    cr cabu
        Clean all and build up to 'pkg1'.

    cr boto
        build only 'pkg1' package, then test only 'pkg1'.

    cr cabuto
        Clean all, build up to 'pkg1', and test only 'pkg1'.


NOTES
    - The 's' verb sets a default package name stored in a configuration file.
    - The 'i' verb runs rosdep install and supports the same specifiers as other verbs.
    - Subsequent commands that require a package argument will use the default if none is provided.
    - Compound verbs can be chained together for streamlined operations.

SEE ALSO
    colcon(1), colcon-clean(1)
"""

import sys
import os
import subprocess
import shlex
from typing import Optional, List

PKG_FILE: str = os.path.expanduser("~/.colcon_shortcuts_pkg")


class ParseError(Exception):
    pass


def _sanitize_pkg_name(pkg: str) -> str:
    """Sanitize package name to prevent path traversal."""
    if not pkg:
        raise ParseError("Package name cannot be empty")
    if "/" in pkg or "\\" in pkg or ".." in pkg:
        raise ParseError("Invalid package name: path traversal detected")
    # Enforce strict rules: only allow alphanumeric, underscores, and dashes
    if not pkg.replace("_", "").replace("-", "").isalnum():
        raise ParseError("Invalid package name: only alphanumeric, underscores, and dashes allowed")
    return pkg


def _find_workspace_root() -> str:
    """Find the workspace root by looking for a 'src' directory.

    Searches from the current directory upward until finding a directory
    containing a 'src' subdirectory, similar to how colcon detects workspaces.

    Returns:
        Absolute path to the workspace root directory.

    Raises:
        ParseError: If no workspace root is found.
    """
    current = os.path.abspath(os.getcwd())

    # Check if we're inside a src directory
    if os.path.basename(current) == "src":
        return os.path.dirname(current)

    # Search upward for a directory containing 'src'
    while True:
        src_path = os.path.join(current, "src")
        if os.path.isdir(src_path):
            return current

        parent = os.path.dirname(current)
        if parent == current:  # Reached filesystem root
            raise ParseError("Could not find workspace root (no 'src' directory found)")
        current = parent


def _parse_verbs(cmds: str):
    """Parse a string like 'boto' into [(verb, spec), ...]."""
    result = []
    i = 0
    while i < len(cmds):
        if cmds[i] not in ("s", "b", "t", "c", "i"):
            raise ParseError(f"unknown command letter '{cmds[i]}'")

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
    return result


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


def _run_tool(tool: str, args: List[str], extra_opts: List[str]) -> None:
    """Run a tool (colcon or rosdep) with the given arguments."""
    # Defensive: ensure all args are strings and not user-controlled shell input
    safe_args = [str(a) for a in args]
    safe_extra_opts = [str(a) for a in extra_opts]
    cmd = [tool] + safe_args + safe_extra_opts
    print("+ " + " ".join(shlex.quote(a) for a in cmd))
    # Use subprocess.run with shell=False for safety
    ret = subprocess.run(cmd, check=False).returncode
    if ret != 0:
        sys.exit(ret)


def _build_cmd(tool: str, verb: str, spec: str, pkg: Optional[str]) -> List[str]:
    """Build command arguments based on tool, verb, spec, and package."""
    if tool == "rosdep":
        # Find workspace root to build correct paths
        workspace_root = _find_workspace_root()
        src_dir = os.path.join(workspace_root, "src")

        # Determine target path based on spec
        if spec == "a":
            # Install for all packages in workspace
            target_path = src_dir
        elif spec in ("o", "u"):
            # Install only for specific package or package and its dependencies
            if not pkg:
                spec_name = "only" if spec == "o" else "upto"
                raise ParseError(f"rosdep '{spec_name}' requires a package name")
            safe_pkg = _sanitize_pkg_name(pkg)
            target_path = os.path.join(src_dir, safe_pkg)
        else:
            raise ParseError(f"unknown specifier '{spec}'")

        # Build command with common flags
        args = ["install", "--from-paths", target_path, "--ignore-src", "-y", "-r", "-v"]
        return args

    # fallback for colcon
    return _build_colcon_cmd(verb, spec, pkg)


def _build_rosdep_cmd(spec: str, pkg: Optional[str]) -> List[str]:
    """Build rosdep command (wrapper for backward compatibility with tests)."""
    return _build_cmd("rosdep", "", spec, pkg)


def main(argv=None) -> None:
    if argv is None:
        argv = sys.argv[1:]
    if len(argv) < 1:
        print(
            "No arguments provided. Running 'colcon build' by default.\nUse '--help' for more options."
        )
        _run_tool("colcon", ["build"], [])
        sys.exit(0)

    # Add --help and -h support
    if argv[0] in ("--help", "-h"):
        print(__doc__)
        sys.exit(0)

    cmds: str = argv[0]
    rest: List[str] = argv[1:]

    # extract override pkg (first non-dash arg)
    override_pkg: Optional[str] = None
    extra_opts: List[str] = []
    for arg in rest:
        if not arg.startswith("-") and override_pkg is None:
            override_pkg = arg
        else:
            extra_opts.append(arg)

    parsed_verbs = _parse_verbs(cmds)

    # Track if rosdep update has been run
    rosdep_updated = False

    # execute each segment
    for verb, spec in parsed_verbs:
        if verb == "s":
            # set default package
            if not override_pkg:
                error("'s' requires a package name")
            save_default_pkg(override_pkg)
            # do not run any tool for 's'
            continue

        # Determine which tool to use based on verb
        tool = "rosdep" if verb == "i" else "colcon"

        # Run rosdep update before first rosdep install
        if tool == "rosdep" and not rosdep_updated:
            if "--dry-run" not in extra_opts:
                print("+ rosdep update")
                ret = subprocess.run(["rosdep", "update"], check=False).returncode
                if ret != 0:
                    sys.exit(ret)
            else:
                print("+ rosdep update")
            rosdep_updated = True

        # Determine if package is needed
        need_pkg: bool = spec in ("o", "u")
        pkg: Optional[str] = get_pkg(override_pkg) if need_pkg else None

        # Warn if package name provided but will be ignored
        if spec == "a" and override_pkg and not override_pkg.startswith("-"):
            verb_name = {"b": "build", "t": "test", "c": "clean", "i": "install"}.get(verb, verb)
            print(
                f"Warning: Package name '{override_pkg}' provided but specifier defaulted to 'all'.",
                file=sys.stderr,
            )
            print(
                f"         Did you mean '{verb}o {override_pkg}' (only) or '{verb}u {override_pkg}' (up-to)?",
                file=sys.stderr,
            )

        # Build command arguments
        args: List[str] = _build_cmd(tool, verb, spec, pkg)

        # Support --dry-run for tests
        if "--dry-run" in extra_opts:
            cmd_args = [tool] + args + extra_opts
            print("+ " + " ".join(shlex.quote(a) for a in cmd_args))
            continue

        # Execute the command
        _run_tool(tool, args, extra_opts)


if __name__ == "__main__":
    main()
