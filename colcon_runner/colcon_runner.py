#!/usr/bin/env python3
"""
CR(1)                         User Commands                        CR(1)

NAME
    cr - Colcon Runner: concise CLI for common colcon tasks.

SYNOPSIS
    cr VERB [PKG] [OPTIONS]
    cr --help | -h
    cr --version | -v
    cr --init-bash
    cr --install-shell-integration

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


OPTIONS
    --help, -h
        Show this help message and exit.

    --version, -v
        Show the version number and exit.

    --init-bash
        Print the bash initialization script to stdout. Intended for
        use in ~/.bashrc as: eval "$(command cr --init-bash)"

    --install-shell-integration
        Add eval "$(command cr --init-bash)" to ~/.bashrc and remove
        any old-style inline shell integration blocks.

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
import traceback
import yaml
from datetime import datetime
from typing import Optional, List, Tuple, NoReturn
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


def _list_packages() -> List[str]:
    """List all colcon package names in the workspace src directory."""
    import xml.etree.ElementTree as ET

    try:
        workspace_root = _find_workspace_root()
    except ParseError:
        return []  # Silent failure for completion

    src_dir = os.path.join(workspace_root, "src")
    if not os.path.isdir(src_dir):
        return []

    packages = []
    for root, _dirs, files in os.walk(src_dir):
        if "package.xml" in files:
            try:
                tree = ET.parse(os.path.join(root, "package.xml"))
                name_elem = tree.find("name")
                if name_elem is not None and name_elem.text:
                    packages.append(name_elem.text.strip())
            except ET.ParseError:
                continue

    return sorted(set(packages))


def _parse_verbs(cmds: str) -> List[Tuple[str, Optional[str]]]:
    """Parse a string like 'boto' into [(verb, spec), ...]."""
    result: List[Tuple[str, Optional[str]]] = []
    i = 0
    while i < len(cmds):
        if cmds[i] not in ("s", "b", "t", "c", "i"):
            raise ParseError(
                f"unknown command letter '{cmds[i]}'\n"
                f"Valid commands: s (set), b (build), t (test), c (clean), i (install)\n"
                f"Use 'cr --help' for more information"
            )

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
    if verb == "c":
        clean_defaults = [
            "--yes",
            "--base-select",
            "build",
            "install",
            "log",
            "test_result",
        ]
        if spec == "a":
            return ["clean", "workspace", *clean_defaults]
        if spec == "o":
            if not pkg:
                raise ParseError(f"{verb} 'only' requires a package name")
            safe_pkg = _sanitize_pkg_name(pkg)
            return ["clean", "packages", *clean_defaults, "--packages-select", safe_pkg]
        if spec == "u":
            if not pkg:
                raise ParseError(f"{verb} 'upto' requires a package name")
            safe_pkg = _sanitize_pkg_name(pkg)
            return ["clean", "packages", *clean_defaults, "--packages-up-to", safe_pkg]
        raise ParseError(f"unknown specifier '{spec}'")

    if verb == "b":
        args = ["build"]
    elif verb == "t":
        args = ["test"]
    else:
        raise ParseError(f"unsupported verb '{verb}'")

    if spec == "o":
        if not pkg:
            raise ParseError(f"{verb} 'only' requires a package name")
        safe_pkg = _sanitize_pkg_name(pkg)
        args.extend(["--packages-select", safe_pkg])
    elif spec == "u":
        if not pkg:
            raise ParseError(f"{verb} 'upto' requires a package name")
        safe_pkg = _sanitize_pkg_name(pkg)
        args.extend(["--packages-up-to", safe_pkg])
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
    safe_pkg = _sanitize_pkg_name(pkg)
    with open(PKG_FILE, "w", encoding="utf-8") as f:
        f.write(safe_pkg)
    print(f"Default package set to '{safe_pkg}'")


def error(msg: str) -> NoReturn:
    print(f"Error: {msg}", file=sys.stderr)
    sys.exit(1)


def get_pkg(override: Optional[str]) -> str:
    if override:
        return override
    if default := load_default_pkg():
        # Sanitize loaded package name for safety
        return _sanitize_pkg_name(default)
    error("no package specified and no default set")


def _handle_rosdep_update(extra_opts: List[str]) -> None:
    """Handle rosdep and apt update if needed."""
    if not _update_needed():
        print("+ sudo apt update (skipped - already run today)")
        print("+ rosdep update (skipped - already run today)")
        return

    if "--dry-run" in extra_opts:
        print("+ sudo apt update")
        print("+ rosdep update")
        return

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


_EVAL_LINE = 'eval "$(command cr --init-bash)"'


def _get_init_bash() -> str:
    """Return the bash initialization script for cr.

    This is printed by ``cr --init-bash`` and evaluated via
    ``eval "$(command cr --init-bash)"`` in the user's bashrc.
    """
    return """\
# Colcon-runner bash completion (generated by cr --init-bash)
_cr_completions() {
    COMPREPLY=()
    local cur="${COMP_WORDS[COMP_CWORD]}"

    # Complete package names at second argument position (cr VERB PKG)
    if [[ $COMP_CWORD -eq 2 ]]; then
        local packages
        packages=$(command cr --list-packages 2>/dev/null)
        COMPREPLY=( $(compgen -W "$packages" -- "$cur") )
    fi
}

complete -F _cr_completions cr"""


def _remove_old_shell_blocks(content: str) -> Tuple[str, bool]:
    """Remove old-style inline shell integration blocks from bashrc content.

    Detects two block types by their marker comments and removes them:
    - Shell integration block: starts with ``# Colcon-runner shell integration for auto-sourcing``
      and ends at the next line containing only ``}``
    - Completion block: starts with ``# Colcon-runner bash completion`` and ends at
      ``complete -F _cr_completions cr``

    Returns:
        A tuple of (cleaned_content, was_modified).
    """
    lines = content.splitlines(True)  # keep line endings
    out: list[str] = []
    modified = False
    i = 0
    while i < len(lines):
        stripped = lines[i].rstrip("\n\r")

        # Detect old shell-integration wrapper block
        if stripped.lstrip() == "# Colcon-runner shell integration for auto-sourcing":
            modified = True
            # Skip until closing brace
            while i < len(lines):
                if lines[i].rstrip("\n\r").rstrip() == "}":
                    i += 1
                    break
                i += 1
            continue

        # Detect old completion block
        if stripped.lstrip() == "# Colcon-runner bash completion":
            modified = True
            # Skip until ``complete -F _cr_completions cr``
            while i < len(lines):
                if lines[i].rstrip("\n\r").rstrip() == "complete -F _cr_completions cr":
                    i += 1
                    break
                i += 1
            continue

        out.append(lines[i])
        i += 1

    # Collapse runs of 3+ blank lines down to 2
    cleaned = "".join(out)
    while "\n\n\n" in cleaned:
        cleaned = cleaned.replace("\n\n\n", "\n\n")

    return cleaned, modified


def _install_shell_integration() -> None:
    """Install shell integration to ~/.bashrc using the eval pattern.

    Adds ``eval "$(command cr --init-bash)"`` to bashrc.  If old-style
    inline blocks are detected they are removed first (migration).
    """
    bashrc_path = os.path.expanduser("~/.bashrc")

    # Check if bashrc exists
    if not os.path.exists(bashrc_path):
        print(f"Creating {bashrc_path}")
        with open(bashrc_path, "w", encoding="utf-8") as f:
            f.write("# .bashrc\n\n")

    # Read current bashrc
    with open(bashrc_path, "r", encoding="utf-8", errors="replace") as f:
        bashrc_content = f.read()

    # Remove old-style inline blocks if present
    cleaned, had_old_blocks = _remove_old_shell_blocks(bashrc_content)

    # Check if eval line already present
    has_eval = _EVAL_LINE in cleaned

    if has_eval and not had_old_blocks:
        print("Shell integration is already installed in ~/.bashrc")
        return

    if not has_eval:
        # Ensure a trailing newline before appending
        if cleaned and not cleaned.endswith("\n"):
            cleaned += "\n"
        cleaned += f"\n{_EVAL_LINE}\n"

    # Write back (full rewrite to support migration)
    with open(bashrc_path, "w", encoding="utf-8") as f:
        f.write(cleaned)

    if had_old_blocks:
        print("Migrated shell integration to eval pattern in ~/.bashrc")
    else:
        print("Shell integration installed to ~/.bashrc")
    print("To activate, run: source ~/.bashrc")
    print("Or start a new terminal session.")


def main(argv=None) -> None:
    if argv is None:
        argv = sys.argv[1:]

    # Handle empty argv case first
    if not argv:
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

    # Print bash init script (for eval in bashrc)
    if argv[0] == "--init-bash":
        print(_get_init_bash())
        sys.exit(0)

    # Add --install-shell-integration support
    if argv[0] == "--install-shell-integration":
        try:
            _install_shell_integration()
            sys.exit(0)
        except (OSError, IOError, PermissionError) as e:
            error(f"OS error while installing shell integration: {e}")
        except KeyboardInterrupt:
            print("\nInterrupted by user", file=sys.stderr)
            sys.exit(130)

        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            error(f"unexpected error: {e}")

    # --list-packages: output package names (one per line) for shell completion
    # This is a hidden flag used by completion scripts, not documented in help
    if argv[0] == "--list-packages":
        for pkg in _list_packages():
            print(pkg)
        sys.exit(0)

    try:
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
                assert override_pkg is not None  # Help type checker after error() check
                save_default_pkg(override_pkg)
                # do not run any tool for 's'
                continue

            # Determine which tool to use based on verb
            tool = "rosdep" if verb == "i" else "colcon"

            # spec should always be set for non-'s' verbs
            assert spec is not None, f"spec is None for verb '{verb}'"

            # Run apt update and rosdep update before first rosdep install (max once per day)
            if tool == "rosdep" and not rosdep_updated:
                _handle_rosdep_update(extra_opts)
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
    except ParseError as e:
        error(str(e))
    except (OSError, IOError, PermissionError) as e:
        # Catch OS-level issues (including file I/O and missing external tools like colcon/rosdep)
        error(f"OS error while running colcon-runner: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user", file=sys.stderr)
        sys.exit(130)
    except Exception as e:
        # Log full traceback for unexpected errors before showing a concise message
        traceback.print_exc(file=sys.stderr)
        error(f"unexpected error: {e}")


if __name__ == "__main__":
    main()
