# Changelog

## colcon_runner

## [0.12.0] - 2026-02-08

### Added
- Package-first CLI argument order: `cr PKG [VERB]` in addition to existing `cr VERB [PKG]`
- Tab completion of package names at position 1 (`cr my_<TAB>`) and position 2
- `default_spec` parameter on `_parse_verbs()` — package-first mode defaults to "u" (up-to)

### Changed
- Verb-first parsing takes priority: if the first argument is a valid verb string it is always treated as a verb, even if a package with the same name exists
- Package lookup (`_list_packages()`) is only called when verb parsing fails, avoiding unnecessary workspace walks
- Bash completion skips package suggestions when the current word starts with `-`
- Updated SYNOPSIS to show both `cr [PKG] [VERB] [OPTIONS]` and `cr [VERB] [PKG] [OPTIONS]`
- Documentation phrasing: "upto" → "up to" throughout

## [0.9.0] - 2026-01-28

### Added
- Shell integration feature for auto-sourcing workspace after builds
- `--install-shell-integration` flag for automatic shell configuration
- Idempotent shell integration installation

### Fixed
- Resolved pylint warnings in shell integration tests
- Addressed PR review feedback for shell integration
- Pre-commit issues

### Changed
- Shell integration now sources ~/.bashrc instead of workspace directly
- Simplified shell integration to always source workspace after successful commands

## [0.2.0]
