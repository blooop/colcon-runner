#!/usr/bin/env python3
"""
CR(1)                         User Commands                        CR(1)

NAME
    cr - Colcon Runner: concise CLI for common colcon tasks.

SYNOPSIS
    cr VERB [PKG] [OPTIONS]
    cr --help | -h
    cr --version | -v

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
import logging
import yaml
from datetime import datetime
from typing import Optional, List
from importlib.metadata import version, PackageNotFoundError

PKG_FILE: str = os.path.expanduser("~/.colcon_shortcuts_pkg")


# Configure logging with colored output for warnings
class ColoredFormatter(logging.Formatter):
    """Custom formatter that adds color to WARNING level messages."""

    YELLOW = "\033[33m"
    RESET = "\033[0m"

    def format(self, record):
        if record.levelno == logging.WARNING and sys.stderr.isatty():
            record.msg = f"{self.YELLOW}{record.msg}{self.RESET}"
        return super().format(record)


# Set up logger
logger = logging.getLogger("colcon_runner")
logger.setLevel(logging.WARNING)
handler = logging.StreamHandler(sys.stderr)
handler.setFormatter(ColoredFormatter("%(message)s"))
logger.addHandler(handler)


def _get_update_cache_file() -> str:
    """Get the path to today's update cache file (for both apt and rosdep)."""
    today = datetime.now().strftime("%Y-%m-%d")
    return f"/tmp/colcon_runner_update_{today}"


def _update_needed() -> bool:
    """Check if update needs to be run (hasn't been run today)."""
    cache_file = _get_update_cache_file()
    return not os.path.exists(cache_file)


def _mark_updated() -> None:
    """Mark that update has been run today."""
    cache_file = _get_update_cache_file()
    # Create the cache file
    with open(cache_file, "w", encoding="utf-8") as f:
        f.write(datetime.now().isoformat())


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
    """Find the workspace root.

    Checks COLCON_DEFAULTS_FILE for a 'base-path' entry first.
    If not found, searches from the current directory upward until finding a directory
    containing a 'src' subdirectory, similar to how colcon detects workspaces.

    Returns:
        Absolute path to the workspace root directory.

    Raises:
        ParseError: If no workspace root is found.
    """
    # Determine potential defaults file paths in order of precedence
    candidates = []

    # 1. Environment variable
    if "COLCON_DEFAULTS_FILE" in os.environ:
        candidates.append(os.environ["COLCON_DEFAULTS_FILE"])

    # 2. $COLCON_HOME/defaults.yaml
    colcon_home = os.environ.get("COLCON_HOME")
    if colcon_home:
        candidates.append(os.path.join(colcon_home, "defaults.yaml"))

    # 3. ~/.colcon/defaults.yaml (default location)
    candidates.append(os.path.expanduser("~/.colcon/defaults.yaml"))

    # Check candidates in order
    logger.warning(f"Searching for workspace root. Checking defaults files: {candidates}")
    for defaults_file in candidates:
        if os.path.isfile(defaults_file):
            try:
                with open(defaults_file, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    # Check for base-paths (plural) first, which is the more modern approach
                    if data and "base-paths" in data.get("build", {}):
                        base_paths = data["build"]["base-paths"]
                        # Ensure base_paths is a list
                        if not isinstance(base_paths, list):
                            base_paths = [base_paths]

                        # Resolve and validate first valid path
                        for base_path in base_paths:
                            # Resolve relative paths relative to the defaults file
                            if not os.path.isabs(base_path):
                                base_path = os.path.join(os.path.dirname(defaults_file), base_path)

                            # Ensure the path exists
                            if os.path.exists(base_path):
                                logger.warning(
                                    f"Found workspace root in defaults file: {base_path}"
                                )
                                return os.path.abspath(base_path)
                            logger.warning(f"Specified base-path does not exist: {base_path}")

                    # Fallback to base-path (singular) for backward compatibility
                    if data and "base-path" in data:
                        base_path = data["base-path"]
                        # Resolve relative paths relative to the defaults file
                        if not os.path.isabs(base_path):
                            base_path = os.path.join(os.path.dirname(defaults_file), base_path)

                        # Ensure the path exists
                        if os.path.exists(base_path):
                            logger.warning(f"Found workspace root in defaults file: {base_path}")
                            return os.path.abspath(base_path)
                        logger.warning(f"Specified base-path does not exist: {base_path}")
            except Exception as e:
                # Log warning but continue to next candidate or fallback
                logger.warning(f"Failed to parse defaults file '{defaults_file}': {e}")

    # If no base-path is found in defaults, fall back to src directory search
    current = os.path.abspath(os.getcwd())
    logger.warning(f"Using current directory as base: {current}")

    # Check if we're inside a src directory
    if os.path.basename(current) == "src":
        logger.warning(f"Current directory is 'src'. Workspace root: {os.path.dirname(current)}")
        return os.path.dirname(current)

    # Search upward for a directory containing 'src'
    while True:
        src_path = os.path.join(current, "src")
        logger.warning(f"Checking for src directory at: {src_path}")
        if os.path.isdir(src_path):
            logger.warning(f"Found 'src' directory. Workspace root: {current}")
            return current

        parent = os.path.dirname(current)
        if parent == current:  # Reached filesystem root
            logger.warning("No 'src' directory found in current path hierarchy")
            raise ParseError(
                f"Could not find workspace root (no 'src' directory found) from {os.path.abspath(os.getcwd())}"
            )
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

    # Suppress DeprecationWarnings for rosdep (pkg_resources warning)
    env = None
    if tool == "rosdep":
        env = os.environ.copy()
        env["PYTHONWARNINGS"] = "ignore::DeprecationWarning"

    # Use subprocess.run with shell=False for safety
    ret = subprocess.run(cmd, check=False, env=env).returncode
    if ret != 0:
        sys.exit(ret)


def _build_cmd(tool: str, verb: str, spec: str, pkg: Optional[str]) -> List[str]:
    """Build command arguments based on tool, verb, spec, and package."""
    if tool == "rosdep":
        # Find workspace root to build correct paths
        workspace_root = _find_workspace_root()

        # Determine target path based on spec
        if spec == "a":
            # Install for all packages in workspace
            target_path = workspace_root
        elif spec in ("o", "u"):
            # Install only for specific package or package and its dependencies
            if not pkg:
                spec_name = "only" if spec == "o" else "upto"
                raise ParseError(f"rosdep '{spec_name}' requires a package name")
            safe_pkg = _sanitize_pkg_name(pkg)
            target_path = os.path.join(workspace_root, safe_pkg)
        else:
            raise ParseError(f"unknown specifier '{spec}'")

        # Build command with common flags
        args = ["install", "--from-paths", target_path, "--ignore-src", "-y", "-r"]
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

    # Add --version support
    if argv[0] in ("--version", "-v"):
        try:
            pkg_version = version("colcon-runner")
            print(f"cr (colcon-runner) version {pkg_version}")
        except PackageNotFoundError:
            print("cr (colcon-runner) version unknown (not installed)")
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

        # Run apt update and rosdep update before first rosdep install (max once per day)
        if tool == "rosdep" and not rosdep_updated:
            if _update_needed():
                if "--dry-run" not in extra_opts:
                    # Run apt update first to refresh package lists
                    print("+ sudo apt update")
                    ret = subprocess.run(["sudo", "apt", "update"], check=False).returncode
                    if ret != 0:
                        print("Warning: sudo apt update failed, continuing with rosdep update")

                    print("+ rosdep update")
                    # Suppress DeprecationWarnings for rosdep
                    env = os.environ.copy()
                    env["PYTHONWARNINGS"] = "ignore::DeprecationWarning"
                    ret = subprocess.run(["rosdep", "update"], check=False, env=env).returncode
                    if ret != 0:
                        sys.exit(ret)
                    _mark_updated()
                else:
                    print("+ sudo apt update")
                    print("+ rosdep update")
            else:
                print("+ sudo apt update (skipped - already run today)")
                print("+ rosdep update (skipped - already run today)")
            rosdep_updated = True

        # Determine if package is needed
        need_pkg: bool = spec in ("o", "u")
        pkg: Optional[str] = get_pkg(override_pkg) if need_pkg else None

        # Warn if package name provided but will be ignored
        if spec == "a" and override_pkg and not override_pkg.startswith("-"):
            logger.warning(
                f"Package name '{override_pkg}' provided but specifier defaulted to 'all'.\n"
                f"         Did you mean '{verb}o {override_pkg}' (only) or '{verb}u {override_pkg}' (up-to)?"
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
