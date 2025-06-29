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

SPECIFIER
    o       only (--packages-select)
    u       upto (--packages-up-to)
    a       all

Each verb must have a specifier after it, and you can chain as many verb-specifier pairs as you want.  You can set a default package to use, for all subsequent commands, or you can specify a package in the command itself.

USAGE EXAMPLES

  Basic Commands:
    cr ba
        Build all packages. (explicit)

    cr bo pkg_1
        Build only 'pkg_1'.

    cr bu pkg_1
        Build upto 'pkg_1' and its dependencies.

    cr ta
        Test all packages.

    cr to pkg_1
        Test only 'pkg_1'.

    cr tu pkg_1
        Test upto 'pkg_1' and its dependencies.

    cr ca
        Clean workspace (build/, install/, log/, and test_result/ directories)

    cr co pkg_1
        Clean only 'pkg_1'.

    cr cu pkg_1
        Clean upto 'pkg_1'.

  Compound Commands:
    cr s pkg1
        Set 'pkg_1' as the default package for subsequent commands.

    cr cabu
        Clean all and build up to 'pkg1'.

    cr boto
        build only 'pkg1' package, then test only 'pkg1'.

    cr cabuto
        Clean all, build up to 'pkg1', and test only 'pkg1'.


NOTES
    - The 's' verb sets a default package name stored in a configuration file.
    - Subsequent commands that require a package argument will use the default if none is provided.
    - Compound verbs can be chained together for streamlined operations.

SEE ALSO
    colcon(1), colcon-clean(1)
"""

import sys
import os
import subprocess
from typing import Optional, List

PKG_FILE: str = os.path.expanduser("~/.colcon_shortcuts_pkg")


class ParseError(Exception):
    pass


def _parse_verbs(cmds: str):
    """Parse a string like 'boto' into [(verb, spec), ...]."""
    result = []
    i = 0
    while i < len(cmds):
        if cmds[i] in ("s", "b", "t", "c"):
            verb = cmds[i]
            if verb == "s":
                result.append((verb, None))
                i += 1
                continue
            if i + 1 >= len(cmds) or cmds[i + 1] not in ("o", "u", "a"):
                raise ParseError(f"verb '{verb}' must be followed by a specifier (o, u, or a)")
            result.append((verb, cmds[i + 1]))
            i += 2
        else:
            raise ParseError(f"unknown command letter '{cmds[i]}'")
    return result


def _build_colcon_cmd(verb, spec, pkg):
    if verb == "b":
        args = ["colcon", "build"]
    elif verb == "t":
        args = ["colcon", "test"]
    elif verb == "c":
        args = [
            "colcon",
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


def run_colcon(args: List[str], extra_opts: List[str]) -> None:
    cmd = ["colcon"] + args + extra_opts
    print("+ " + " ".join(cmd))
    ret = subprocess.call(cmd)
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

    # extract override pkg (first non-dash arg)
    override_pkg: Optional[str] = None
    extra_opts: List[str] = []
    for arg in rest:
        if not arg.startswith("-") and override_pkg is None:
            override_pkg = arg
        else:
            extra_opts.append(arg)

    parsed_verbs = _parse_verbs(cmds)

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

        run_colcon(args, extra_opts)


if __name__ == "__main__":
    main()
