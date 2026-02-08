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

### Enable Auto-Source After Commands (Optional)

To automatically re-source your environment after any successful `cr` command, install the shell integration:

```bash
cr --install-shell-integration
source ~/.bashrc  # Or start a new terminal
```

This will automatically re-source your workspace after any successful `cr` command, ensuring your environment always reflects the latest workspace state while preserving your ROS installation and any underlay workspaces.

The installation is idempotent - running it multiple times is safe and won't create duplicates.

```
CR(1)                         User Commands                        CR(1)

NAME
    cr - Colcon Runner: concise CLI for common colcon tasks.

SYNOPSIS
    cr [PKG] [VERB]
    cr --help | -h
    cr --version | -v
    cr --install-shell-integration

DESCRIPTION
    A minimal wrapper around colcon providing short, mnemonic commands
    for build, test, clean, and package selection operations.

    If the first argument matches a known package in the workspace, it
    is used as the target package. Otherwise it is treated as a verb
    string.

STATE
    s       set a default package for subsequent commands.

VERBS
    b       build packages.
    t       test packages.
    c       clean packages.
    i       install dependencies using rosdep.

SPECIFIER
    o       only (--packages-select)
    u       upto (--packages-up-to)
    a       all

    If a package is given, the default specifier is "u" (up-to).
    If no package is given, the default specifier is "a" (all).

USAGE EXAMPLES

  No arguments:
    cr
        Build all packages. (default action when no arguments provided)

  Package only (tab-completable):
    cr pkg_1
        Build upto 'pkg_1' and its dependencies. (default action)

  Package with verb:
    cr pkg_1 b
        Build upto 'pkg_1' and its dependencies.

    cr pkg_1 bo
        Build only 'pkg_1'.

    cr pkg_1 bu
        Build upto 'pkg_1' and its dependencies. (explicit)

    cr pkg_1 t
        Test upto 'pkg_1' and its dependencies.

    cr pkg_1 to
        Test only 'pkg_1'.

    cr pkg_1 c
        Clean upto 'pkg_1'.

    cr pkg_1 co
        Clean only 'pkg_1'.

    cr pkg_1 ca
        Clean all. (explicit "a" overrides package default)

    cr pkg_1 i
        Install dependencies for 'pkg_1' and its dependencies.

    cr pkg_1 io
        Install dependencies only for 'pkg_1'.

    cr pkg_1 s
        Set 'pkg_1' as the default package for subsequent commands.

  Verb only (operates on all packages):
    cr b
        Build all packages.

    cr t
        Test all packages.

    cr c
        Clean workspace (build/, install/, log/, and test_result/).

    cr i
        Install all dependencies using rosdep.

  Compound commands:
    cr pkg_1 bt
        Build upto 'pkg_1', then test upto 'pkg_1'.

    cr pkg_1 boto
        Build only 'pkg_1', then test only 'pkg_1'.

    cr pkg_1 cbt
        Clean upto, build upto, test upto 'pkg_1'.

    cr cbt
        Clean all, build all, test all.

    cr pkg_1 cabuto
        Clean all, build upto 'pkg_1', test only 'pkg_1'.

OPTIONS
    --help, -h
        Show this help message and exit.

    --version, -v
        Show the version number and exit.

    --install-shell-integration
        Install bash shell integration to ~/.bashrc for auto-sourcing
        after successful cr commands and tab completion of package names.

NOTES
    - If the first argument matches a known package in the workspace,
      it is used as the target package with a default specifier of "u"
      (up-to). Otherwise it is parsed as a verb string with a default
      specifier of "a" (all).
    - The 's' verb sets a default package name stored in a config file.
    - The 'i' verb runs rosdep install and supports the same specifiers.
    - Subsequent commands that require a package argument will use the
      default if none is provided.
    - Compound verbs can be chained for streamlined operations.
    - Tab completion for package names is available when shell
      integration is installed.

SEE ALSO
    colcon(1), colcon-clean(1)
```

Colcon runner assumes you have colcon defaults set up to ensure your paths and settings are applied when you run colcon.  This is an example of a colcon defaults file to get consistent behavior across the commands supported here:

```yaml
build:
  symlink-install: true
  base-paths:
    - "/home/ros_ws/src"
  build-base: "/home/ros_ws/ros_build/build"
  install-base: "/home/ros_ws/ros_build/install"
  cmake-args:
    - -DCMAKE_BUILD_TYPE=RelWithDebInfo
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

test:
  build-base: "/home/ros_ws/ros_build/build"
  install-base: "/home/ros_ws/ros_build/install"
  log-base: "/home/ros_ws/ros_build/logs"
  event-handlers:
    - console_direct+

test-result:
  test-result-base: "/home/ros_ws/ros_build/build"

clean.workspace:
  "yes": true
  base-select:
    - build
    - install
    - log
    - test_result
  build-base: "/home/ros_ws/ros_build/build"
  install-base: "/home/ros_ws/ros_build/install"
  log-base: "/home/ros_ws/ros_build/logs"
  test-result-base: "/home/ros_ws/ros_build/build"

clean.packages:
  "yes": true
  base-select:
    - build
    - install
    - log
    - test_result
  build-base: "/home/ros_ws/ros_build/build"
  install-base: "/home/ros_ws/ros_build/install"
  log-base: "/home/ros_ws/ros_build/logs"
  test-result-base: "/home/ros_ws/ros_build/build"

'':
  log-base: "/home/ros_ws/ros_build/logs"
```
