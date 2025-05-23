# colcon_runner
A template repo for python projects that is set up using [pixi](https://pixi.sh). 

This has basic setup for

* pylint
* ruff
* black
* pytest
* git-lfs
* basic github actions ci
* pulling updates from this template
* codecov
* pypi upload
* dependabot

## Continuous Integration Status

[![Ci](https://github.com/blooop/colcon_runner/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/blooop/colcon_runner/actions/workflows/ci.yml?query=branch%3Amain)
[![Codecov](https://codecov.io/gh/blooop/colcon_runner/branch/main/graph/badge.svg?token=Y212GW1PG6)](https://codecov.io/gh/blooop/colcon_runner)
[![GitHub issues](https://img.shields.io/github/issues/blooop/colcon_runner.svg)](https://GitHub.com/blooop/colcon_runner/issues/)
[![GitHub pull-requests merged](https://badgen.net/github/merged-prs/blooop/colcon_runner)](https://github.com/blooop/colcon_runner/pulls?q=is%3Amerged)
[![GitHub release](https://img.shields.io/github/release/blooop/colcon_runner.svg)](https://GitHub.com/blooop/colcon_runner/releases/)
[![License](https://img.shields.io/github/license/blooop/colcon_runner)](https://opensource.org/license/mit/)
[![Python](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue)](https://www.python.org/downloads/)
[![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

```
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
    cr 
        Build all packages. (default action)

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
        Clean all (build/, install/, and log/ directories)

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
```
