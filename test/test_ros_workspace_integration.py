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
    defaults_text = DEFAULTS_TEMPLATE.read_text(encoding="utf-8").format(
        workspace=str(workspace)
    )
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
