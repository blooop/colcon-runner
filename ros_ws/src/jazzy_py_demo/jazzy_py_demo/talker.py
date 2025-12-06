"""Minimal helper used by the colcon-runner integration workspace."""

from __future__ import annotations


def compose_greeting(name: str) -> str:
    """Create a predictable greeting string."""
    safe_name = name.strip() or "world"
    return f"Hello, {safe_name}! Welcome to ROS Jazzy."


def main() -> None:
    print(compose_greeting("world"))


if __name__ == "__main__":
    main()
