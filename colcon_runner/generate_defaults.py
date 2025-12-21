#!/usr/bin/env python3
"""Generate colcon defaults files with YAML anchors for workspace configuration."""

import sys
import os
import argparse
from typing import Optional


def generate_colcon_defaults(workspace_path: str) -> str:
    """
    Generate a colcon defaults YAML configuration using anchors.

    Args:
        workspace_path: The root path of the colcon workspace

    Returns:
        A string containing the YAML configuration with anchors
    """
    # Normalize the workspace path to remove trailing slashes
    workspace_path = os.path.normpath(workspace_path)

    # Create the YAML content using anchors to avoid repetition
    yaml_content = f"""# Colcon defaults file generated for workspace: {workspace_path}
# Uses YAML anchors (&) and aliases (*) to avoid repetition

# Define common paths as anchors
paths: &paths
  base-paths: ["{workspace_path}/src"]
  build-base: &build-base "{workspace_path}/build"
  install-base: &install-base "{workspace_path}/install"
  log-base: &log-base "{workspace_path}/log"
  test-result-base: &test-result-base "{workspace_path}/build"

# Build configuration
build:
  symlink-install: true
  base-paths: ["{workspace_path}/src"]
  build-base: *build-base
  install-base: *install-base
  cmake-args:
    - "-DCMAKE_BUILD_TYPE=RelWithDebInfo"
    - "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

# Test configuration
test:
  build-base: *build-base
  install-base: *install-base
  log-base: *log-base
  event-handlers: ["console_direct+"]

# Test result configuration
test-result:
  test-result-base: *test-result-base

# Clean workspace configuration
clean.workspace: &clean-config
  "yes": true
  base-select: ["build", "install", "log", "test_result"]
  build-base: *build-base
  install-base: *install-base
  log-base: *log-base
  test-result-base: *test-result-base

# Clean packages configuration (reuses clean.workspace config)
clean.packages: *clean-config

# Default configuration for all commands
"":
  log-base: *log-base
"""

    return yaml_content


def main(argv: Optional[list] = None) -> None:
    """
    Main entry point for the generate-colcon-defaults executable.

    Args:
        argv: Command line arguments (defaults to sys.argv[1:])
    """
    if argv is None:
        argv = sys.argv[1:]

    parser = argparse.ArgumentParser(
        description="Generate a colcon defaults file with YAML anchors",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate defaults for current directory
  generate-colcon-defaults .

  # Generate defaults for specific workspace
  generate-colcon-defaults /path/to/workspace

  # Generate and save to a file
  generate-colcon-defaults ~/ros_ws > ~/.colcon/defaults.yaml

  # Generate for absolute path
  generate-colcon-defaults /home/user/colcon_ws
"""
    )

    parser.add_argument(
        "workspace",
        nargs="?",
        default=".",
        help="Path to the colcon workspace root (default: current directory)"
    )

    parser.add_argument(
        "-o", "--output",
        help="Output file path (default: prints to stdout)"
    )

    args = parser.parse_args(argv)

    # Resolve the workspace path to an absolute path
    workspace_path = os.path.abspath(os.path.expanduser(args.workspace))

    # Verify the workspace path exists
    if not os.path.exists(workspace_path):
        print(f"Error: Workspace path does not exist: {workspace_path}", file=sys.stderr)
        sys.exit(1)

    # Generate the defaults content
    defaults_content = generate_colcon_defaults(workspace_path)

    # Output the content
    if args.output:
        output_path = os.path.expanduser(args.output)
        # Create parent directory if it doesn't exist
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(defaults_content)
        print(f"Generated colcon defaults file: {output_path}", file=sys.stderr)
    else:
        print(defaults_content)


if __name__ == "__main__":
    main()
