# Changelog

## colcon_runner

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
