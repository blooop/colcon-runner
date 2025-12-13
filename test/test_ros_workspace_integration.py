import os
import shutil
import subprocess
import sys
from pathlib import Path

WORKSPACE_SOURCE = Path(__file__).parent.parent / "ros_ws"
DEFAULTS_TEMPLATE = Path(__file__).parent / "colcon_defaults_template.yaml"


def _run_cr(args, workspace, env):
    cmd = [sys.executable, "-m", "colcon_runner.colcon_runner", *args]
    result = subprocess.run(
        cmd, cwd=workspace, env=env, check=False, capture_output=True, text=True
    )
    if result.returncode != 0:
        raise AssertionError(
            f"Command {' '.join(args)} failed with {result.returncode}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    if "Cannot find file" in (result.stderr or ""):
        raise AssertionError(f"Unexpected colcon stderr: {result.stderr}")
    return result


def _prepare_env(tmp_path):
    env = os.environ.copy()
    env["HOME"] = str(tmp_path / "home")
    env["ROS_DISTRO"] = env.get("ROS_DISTRO", "jazzy")
    env.pop("COLCON_DEFAULTS_FILE", None)
    (tmp_path / "home").mkdir(parents=True, exist_ok=True)
    return env


def _write_defaults(workspace: Path, tmp_path: Path) -> Path:
    defaults_text = DEFAULTS_TEMPLATE.read_text(encoding="utf-8").format(workspace=str(workspace))
    defaults_path = tmp_path / "colcon_defaults.yaml"
    defaults_path.write_text(defaults_text, encoding="utf-8")
    return defaults_path


def test_colcon_runner_builds_tests_and_cleans(tmp_path):
    workspace = tmp_path / "ros_ws"
    shutil.copytree(WORKSPACE_SOURCE, workspace)

    env = _prepare_env(tmp_path)
    env["COLCON_DEFAULTS_FILE"] = str(_write_defaults(workspace, tmp_path))

    # Ensure workspace starts clean, build, test, then clean again
    _run_cr(["c"], workspace, env)
    _run_cr(["b"], workspace, env)
    _run_cr(["t"], workspace, env)

    # Verify artifacts from both packages
    assert (workspace / "install").exists()
    assert (workspace / "build" / "jazzy_py_demo" / "pytest.xml").exists()
    assert (workspace / "build" / "jazzy_cpp_demo" / "Testing").exists()

    _run_cr(["c"], workspace, env)
    for path in ("build", "install", "log", "test_result"):
        assert not (workspace / path).exists()


def test_workspace_sourcing_after_build(tmp_path):
    """Verify that the workspace can be sourced and packages are accessible."""
    workspace = tmp_path / "ros_ws"
    shutil.copytree(WORKSPACE_SOURCE, workspace)

    env = _prepare_env(tmp_path)
    env["COLCON_DEFAULTS_FILE"] = str(_write_defaults(workspace, tmp_path))

    # Build the workspace
    _run_cr(["b"], workspace, env)

    # Verify setup files exist
    setup_bash = workspace / "install" / "setup.bash"
    setup_sh = workspace / "install" / "setup.sh"
    assert setup_bash.exists(), "setup.bash should exist after build"
    assert setup_sh.exists(), "setup.sh should exist after build"

    # Test that sourcing the workspace sets up the environment correctly
    # We need to run commands in a shell that sources the setup file first
    source_cmd = f"source {setup_bash} && env"
    result = subprocess.run(
        ["bash", "-c", source_cmd],
        cwd=workspace,
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )

    # Parse the environment variables from the output
    sourced_env = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            key, _, value = line.partition("=")
            sourced_env[key] = value

    # Verify critical colcon/ROS environment variables are set
    # Check for either AMENT_PREFIX_PATH (ROS 2) or CMAKE_PREFIX_PATH (older colcon)
    # or COLCON_PREFIX_PATH (colcon-specific)
    prefix_path_vars = ["AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH"]
    found_prefix_var = None
    for var in prefix_path_vars:
        if var in sourced_env and str(workspace / "install") in sourced_env[var]:
            found_prefix_var = var
            break

    if not found_prefix_var:
        # Print available environment variables for debugging
        env_vars_with_path = {
            k: v for k, v in sourced_env.items() if "path" in k.lower() or "prefix" in k.lower()
        }
        raise AssertionError(
            f"No workspace prefix path variable found. Checked: {prefix_path_vars}\n"
            f"PATH-related variables found:\n{env_vars_with_path}"
        )

    # Verify Python path includes the workspace
    assert "PYTHONPATH" in sourced_env, "PYTHONPATH should be set after sourcing"
    pythonpath_ok = (
        "jazzy_py_demo" in sourced_env["PYTHONPATH"]
        or str(workspace / "install") in sourced_env["PYTHONPATH"]
    )
    if not pythonpath_ok:
        raise AssertionError(
            f"PYTHONPATH does not include workspace packages.\n"
            f"PYTHONPATH={sourced_env['PYTHONPATH']}"
        )

    # Test that Python packages can be imported after sourcing
    import_cmd = f"source {setup_bash} && python -c 'import jazzy_py_demo; from jazzy_py_demo.talker import compose_greeting; print(compose_greeting(\"test\"))'"
    result = subprocess.run(
        ["bash", "-c", import_cmd],
        cwd=workspace,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    if result.returncode != 0:
        raise AssertionError(
            f"Failed to import jazzy_py_demo after sourcing workspace\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )

    assert (
        "Hello, test!" in result.stdout
    ), "Package import and execution should work after sourcing"
