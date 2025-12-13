# colcon-runner

## Continuous Integration Status

[![Ci](https://github.com/blooop/colcon-runner/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/blooop/colcon-runner/actions/workflows/ci.yml?query=branch%3Amain)
[![Codecov](https://codecov.io/gh/blooop/colcon-runner/branch/main/graph/badge.svg?token=Y212GW1PG6)](https://codecov.io/gh/blooop/colcon-runner)
[![GitHub issues](https://img.shields.io/github/issues/blooop/colcon-runner.svg)](https://GitHub.com/blooop/colcon-runner/issues/)
[![GitHub pull-requests merged](https://badgen.net/github/merged-prs/blooop/colcon-runner)](https://github.com/blooop/colcon-runner/pulls?q=is%3Amerged)
[![GitHub release](https://img.shields.io/github/release/blooop/colcon-runner.svg)](https://GitHub.com/blooop/colcon-runner/releases/)
[![License](https://img.shields.io/github/license/blooop/colcon-runner)](https://opensource.org/license/mit/)
[![Python](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue)](https://www.python.org/downloads/)
[![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

Colcon runner is a minimal CLI wrapper for [colcon](https://colcon.readthedocs.io/en/released/) that provides concise and flexible commands for common build, test, and clean tasks in ROS workspaces. It supports [colcon defaults](https://colcon.readthedocs.io/en/released/user/configuration.html#colcon-defaults-yaml) for consistent configuration.

## Installation

You can install colcon-runner using pip:

```bash
pip install colcon-runner
```

```
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
    cr
        Build all packages. (default action when no arguments provided)

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
```

Colcon runner assumes you have colcon defaults set up to ensure your paths and settings are applied when you run colcon.  This is an example of a colcon defaults file to get consistent behavior across the commands supported here:

```yaml
{
  "build": {
    "symlink-install": true,
    "base-paths": ["/home/ros_ws/src"],
    "build-base": "/home/ros_ws/ros_build/build",
    "install-base": "/home/ros_ws/ros_build/install",
    "cmake-args": [
      "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    ]
  },
  "test": {
    "build-base": "/home/ros_ws/ros_build/build",
    "install-base": "/home/ros_ws/ros_build/install",
    "log-base": "/home/ros_ws/ros_build/logs",
    "event-handlers": ["console_direct+"]
  },
  "test-result": {
    "test-result-base": "/home/ros_ws/ros_build/build"
  },
  "clean.workspace": {
    "yes": true,
    "base-select": ["build", "install", "log", "test_result"],
    "build-base": "/home/ros_ws/ros_build/build",
    "install-base": "/home/ros_ws/ros_build/install",
    "log-base": "/home/ros_ws/ros_build/logs",
    "test-result-base": "/home/ros_ws/ros_build/build"
  },
  "clean.packages": {
    "yes": true,
    "base-select": ["build", "install", "log", "test_result"],
    "build-base": "/home/ros_ws/ros_build/build",
    "install-base": "/home/ros_ws/ros_build/install",
    "log-base": "/home/ros_ws/ros_build/logs",
    "test-result-base": "/home/ros_ws/ros_build/build"
  },
  "": {"log-base": "/home/ros_ws/ros_build/logs"}
}
```
