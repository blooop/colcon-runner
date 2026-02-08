# ==================================================
# test_cr.py – basic unit tests for the *cr* module
# ==================================================

import contextlib
import io
import tempfile
import unittest
import os
import shutil
from unittest import mock
from importlib.metadata import version

# Import the module under test
from colcon_runner import colcon_runner


class ParseVerbTests(unittest.TestCase):
    # pylint: disable=protected-access
    def test_valid_pairs(self):
        self.assertEqual(colcon_runner._parse_verbs("ba"), [("b", "a")])
        self.assertEqual(colcon_runner._parse_verbs("boto"), [("b", "o"), ("t", "o")])

    def test_default_specifier(self):
        # When no specifier is provided, default to "a" (all)
        self.assertEqual(colcon_runner._parse_verbs("b"), [("b", "a")])
        self.assertEqual(colcon_runner._parse_verbs("t"), [("t", "a")])
        self.assertEqual(colcon_runner._parse_verbs("c"), [("c", "a")])

    def test_unknown_verb(self):
        with self.assertRaises(colcon_runner.ParseError):
            colcon_runner._parse_verbs("xa")

    def test_unknown_spec_defaults_then_fails_on_unknown_verb(self):
        # "bz" parses "b" with default "a", then fails on unknown verb "z"
        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._parse_verbs("bz")
        self.assertIn("unknown command letter 'z'", str(cm.exception))

    def test_default_in_compound_commands(self):
        # Test default specifier in compound commands
        self.assertEqual(colcon_runner._parse_verbs("bt"), [("b", "a"), ("t", "a")])
        self.assertEqual(colcon_runner._parse_verbs("bto"), [("b", "a"), ("t", "o")])
        self.assertEqual(colcon_runner._parse_verbs("bobt"), [("b", "o"), ("b", "a"), ("t", "a")])
        self.assertEqual(colcon_runner._parse_verbs("cbt"), [("c", "a"), ("b", "a"), ("t", "a")])

    def test_install_verb(self):
        # 'i' verb takes specifiers like 'b', 't', 'c'
        self.assertEqual(colcon_runner._parse_verbs("i"), [("i", "a")])
        self.assertEqual(colcon_runner._parse_verbs("ia"), [("i", "a")])
        self.assertEqual(colcon_runner._parse_verbs("io"), [("i", "o")])
        self.assertEqual(colcon_runner._parse_verbs("iu"), [("i", "u")])
        # 'i' can be combined with other verbs
        self.assertEqual(colcon_runner._parse_verbs("ib"), [("i", "a"), ("b", "a")])
        self.assertEqual(colcon_runner._parse_verbs("iobo"), [("i", "o"), ("b", "o")])
        self.assertEqual(colcon_runner._parse_verbs("ibt"), [("i", "a"), ("b", "a"), ("t", "a")])

    def test_install_verb_invalid_specifiers(self):
        # Test that invalid specifiers default to "a" for 'i' verb, then 'x' is treated as next verb
        # Since 'x' is not a valid verb, it should raise an error
        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._parse_verbs("ix")
        self.assertIn("unknown command letter 'x'", str(cm.exception))

        # Test another invalid character
        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._parse_verbs("iz")
        self.assertIn("unknown command letter 'z'", str(cm.exception))

        # Test that valid verbs after invalid specifier work
        # 'ib' should parse as: 'i' with default 'a', then 'b' with default 'a'
        self.assertEqual(colcon_runner._parse_verbs("ib"), [("i", "a"), ("b", "a")])


class BuildCommandTests(unittest.TestCase):
    # pylint: disable=protected-access
    def test_build_all(self):
        cmd = colcon_runner._build_colcon_cmd("b", "a", None)
        self.assertEqual(cmd, ["build"])

    def test_test_only(self):
        cmd = colcon_runner._build_colcon_cmd("t", "o", "pkg")
        self.assertEqual(cmd, ["test", "--packages-select", "pkg"])

    def test_missing_pkg(self):
        with self.assertRaises(colcon_runner.ParseError):
            colcon_runner._build_colcon_cmd("c", "u", None)

    def test_clean_only_missing_pkg(self):
        # Test that clean only without package raises ParseError
        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._build_colcon_cmd("c", "o", None)
        self.assertIn("'only' requires a package name", str(cm.exception))

    def test_clean_upto_missing_pkg(self):
        # Test that clean upto without package raises ParseError
        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._build_colcon_cmd("c", "u", None)
        self.assertIn("'upto' requires a package name", str(cm.exception))

    def test_clean_all(self):
        cmd = colcon_runner._build_colcon_cmd("c", "a", None)
        self.assertEqual(
            cmd,
            [
                "clean",
                "workspace",
                "--yes",
                "--base-select",
                "build",
                "install",
                "log",
                "test_result",
            ],
        )

    def test_clean_only(self):
        cmd = colcon_runner._build_colcon_cmd("c", "o", "pkg")
        self.assertEqual(
            cmd,
            [
                "clean",
                "packages",
                "--yes",
                "--base-select",
                "build",
                "install",
                "log",
                "test_result",
                "--packages-select",
                "pkg",
            ],
        )

    def test_clean_upto(self):
        cmd = colcon_runner._build_colcon_cmd("c", "u", "pkg")
        self.assertEqual(
            cmd,
            [
                "clean",
                "packages",
                "--yes",
                "--base-select",
                "build",
                "install",
                "log",
                "test_result",
                "--packages-up-to",
                "pkg",
            ],
        )


class RosdepCommandTests(unittest.TestCase):
    # pylint: disable=protected-access
    def setUp(self):
        # Mock workspace root detection to return a known path
        self.workspace_patch = mock.patch.object(
            colcon_runner, "_find_workspace_root", return_value="/fake/workspace"
        )
        self.workspace_patch.start()
        self.addCleanup(self.workspace_patch.stop)

    def test_install_all(self):
        cmd = colcon_runner._build_rosdep_cmd("a", None)
        self.assertEqual(
            cmd, ["install", "--from-paths", "/fake/workspace", "--ignore-src", "-y", "-r"]
        )

    def test_install_only(self):
        cmd = colcon_runner._build_rosdep_cmd("o", "pkg")
        self.assertEqual(
            cmd, ["install", "--from-paths", "/fake/workspace/pkg", "--ignore-src", "-y", "-r"]
        )

    def test_install_upto(self):
        cmd = colcon_runner._build_rosdep_cmd("u", "pkg")
        self.assertEqual(
            cmd, ["install", "--from-paths", "/fake/workspace/pkg", "--ignore-src", "-y", "-r"]
        )

    def test_missing_pkg_only(self):
        with self.assertRaises(colcon_runner.ParseError):
            colcon_runner._build_rosdep_cmd("o", None)

    def test_missing_pkg_upto(self):
        with self.assertRaises(colcon_runner.ParseError):
            colcon_runner._build_rosdep_cmd("u", None)

    def test_package_name_sanitization(self):
        # Test valid package names
        self.assertEqual(colcon_runner._sanitize_pkg_name("valid_pkg"), "valid_pkg")
        self.assertEqual(colcon_runner._sanitize_pkg_name("pkg-with-dash"), "pkg-with-dash")
        self.assertEqual(colcon_runner._sanitize_pkg_name("pkg123"), "pkg123")

        # Test path traversal attempts
        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._sanitize_pkg_name("../etc/passwd")
        self.assertIn("path traversal", str(cm.exception))

        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._sanitize_pkg_name("pkg/../other")
        self.assertIn("path traversal", str(cm.exception))

        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._sanitize_pkg_name("pkg/subdir")
        self.assertIn("path traversal", str(cm.exception))

        # Test invalid characters
        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._sanitize_pkg_name("pkg@invalid")
        self.assertIn("only alphanumeric", str(cm.exception))

        # Test empty package name
        with self.assertRaises(colcon_runner.ParseError) as cm:
            colcon_runner._sanitize_pkg_name("")
        self.assertIn("cannot be empty", str(cm.exception))


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        # Use a temporary config location so we don't clobber the user's file.
        # Use tempfile.mkstemp to create a file descriptor and path we can use
        fd, self.tmp_file_path = tempfile.mkstemp()
        os.close(fd)  # Close file descriptor but keep the file
        self.addCleanup(
            lambda: os.path.exists(self.tmp_file_path) and os.unlink(self.tmp_file_path)
        )
        os.environ["CR_CONFIG"] = self.tmp_file_path

        # Mock workspace root detection to return a known path
        self.workspace_patch = mock.patch.object(
            colcon_runner, "_find_workspace_root", return_value="/test/workspace"
        )
        self.workspace_patch.start()
        self.addCleanup(self.workspace_patch.stop)

    def test_set_default_and_dry_run(self):
        # Patch subprocess.run so no real commands are executed.
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            # Set default package
            colcon_runner.main(["s", "demo_pkg"])

            # Use a public method or mock here if one exists, but for tests it's acceptable to
            # test internal functionality
            # pylint: disable=protected-access
            self.assertEqual(colcon_runner.load_default_pkg(), "demo_pkg")

            # Issue a compound command using the default + dry‑run
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["boto", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("--packages-select demo_pkg", output)
            # subprocess.run should *not* be called when --dry-run is active
            m_sp.run.assert_not_called()

    def test_install_verb_dry_run(self):
        # Test that 'i' verb generates correct rosdep command
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            # Test install all
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["ia", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("rosdep update", output)
            self.assertIn("rosdep install --from-paths /test/workspace --ignore-src -y -r", output)

            # Test install only with package
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["io", "test_pkg", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("rosdep update", output)
            self.assertIn(
                "rosdep install --from-paths /test/workspace/test_pkg --ignore-src -y -r",
                output,
            )

            # subprocess.run should *not* be called when --dry-run is active
            m_sp.run.assert_not_called()

    def test_warning_package_with_default_spec(self):
        # Test that warning is shown when package name provided but spec defaults to 'all'
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            # Test build with package but no specifier (defaults to 'all')
            with self.assertLogs(colcon_runner.logger, level="WARNING") as cm:
                buf_stdout = io.StringIO()
                with contextlib.redirect_stdout(buf_stdout):
                    colcon_runner.main(["b", "example_package", "--dry-run"])

            log_output = "\n".join(cm.output)
            self.assertIn("Package name 'example_package' provided", log_output)
            self.assertIn("Did you mean 'bo example_package' (only)", log_output)
            self.assertIn("or 'bu example_package' (up-to)?", log_output)

            # Test install with package but no specifier
            with self.assertLogs(colcon_runner.logger, level="WARNING") as cm:
                buf_stdout = io.StringIO()
                with contextlib.redirect_stdout(buf_stdout):
                    colcon_runner.main(["i", "example_package", "--dry-run"])

            log_output = "\n".join(cm.output)
            self.assertIn("Package name 'example_package' provided", log_output)
            self.assertIn("Did you mean 'io example_package' (only)", log_output)
            self.assertIn("or 'iu example_package' (up-to)?", log_output)

            # subprocess.run should *not* be called when --dry-run is active
            m_sp.run.assert_not_called()

    def test_clean_all_dry_run(self):
        # Test that 'cr c' defaults to clean all (workspace)
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["c", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("colcon clean workspace", output)
            self.assertIn("--yes", output)
            self.assertIn("--base-select build install log test_result", output)
            m_sp.run.assert_not_called()

    def test_clean_all_explicit_dry_run(self):
        # Test that 'cr ca' explicitly cleans all (workspace)
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["ca", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("colcon clean workspace", output)
            self.assertIn("--yes", output)
            self.assertIn("--base-select build install log test_result", output)
            m_sp.run.assert_not_called()

    def test_clean_only_dry_run(self):
        # Test that 'cr co pkg' cleans only specific package
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["co", "test_pkg", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("colcon clean packages", output)
            self.assertIn("--packages-select test_pkg", output)
            self.assertIn("--yes", output)
            m_sp.run.assert_not_called()

    def test_clean_upto_dry_run(self):
        # Test that 'cr cu pkg' cleans up to specific package
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["cu", "test_pkg", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("colcon clean packages", output)
            self.assertIn("--packages-up-to test_pkg", output)
            self.assertIn("--yes", output)
            m_sp.run.assert_not_called()


class UpdateCacheTests(unittest.TestCase):
    # pylint: disable=protected-access
    def setUp(self):
        # Mock workspace root detection
        self.workspace_patch = mock.patch.object(
            colcon_runner, "_find_workspace_root", return_value="/test/workspace"
        )
        self.workspace_patch.start()
        self.addCleanup(self.workspace_patch.stop)

    def test_rosdep_cache_file_format(self):
        # Test that cache file has correct date format
        cache_file = colcon_runner._get_update_cache_file()
        self.assertIn("/tmp/colcon_runner_update_", cache_file)
        # Should end with date in YYYY-MM-DD format
        import re

        self.assertTrue(re.search(r"\d{4}-\d{2}-\d{2}$", cache_file))

    def test_rosdep_update_needed_no_cache(self):
        # When cache file doesn't exist, update is needed
        with mock.patch.object(colcon_runner.os.path, "exists", return_value=False):
            self.assertTrue(colcon_runner._update_needed())

    def test_rosdep_update_not_needed_with_cache(self):
        # When cache file exists, update is not needed
        with mock.patch.object(colcon_runner.os.path, "exists", return_value=True):
            self.assertFalse(colcon_runner._update_needed())

    def test_rosdep_update_runs_once_then_skipped(self):
        # Test that rosdep update runs first time, then is skipped
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            with mock.patch.object(colcon_runner, "_update_needed") as m_update_needed:
                with mock.patch.object(colcon_runner, "_mark_updated"):
                    # First call - update is needed
                    m_update_needed.return_value = True

                    buf = io.StringIO()
                    with contextlib.redirect_stdout(buf):
                        colcon_runner.main(["ia", "--dry-run"])

                    output = buf.getvalue()
                    self.assertIn("sudo apt update", output)
                    self.assertIn("rosdep update", output)
                    self.assertNotIn("skipped", output)

                    # Second call - update not needed
                    m_update_needed.return_value = False

                    buf = io.StringIO()
                    with contextlib.redirect_stdout(buf):
                        colcon_runner.main(["ia", "--dry-run"])

                    output = buf.getvalue()
                    self.assertIn("sudo apt update (skipped - already run today)", output)
                    self.assertIn("rosdep update (skipped - already run today)", output)


class WorkspaceRootTests(unittest.TestCase):
    # pylint: disable=protected-access
    def setUp(self):
        # Create a temporary directory structure
        self.test_dir = tempfile.mkdtemp()
        self.addCleanup(lambda: os.path.exists(self.test_dir) and shutil.rmtree(self.test_dir))

    def test_find_root_with_defaults_file(self):
        workspace_dir = os.path.join(self.test_dir, "custom_workspace")
        os.makedirs(workspace_dir)
        defaults_path = os.path.join(self.test_dir, "defaults.yaml")
        with open(defaults_path, "w", encoding="utf-8") as f:
            f.write(f"base-path: {workspace_dir}\n")

        with mock.patch.dict(os.environ, {"COLCON_DEFAULTS_FILE": defaults_path}):
            root = colcon_runner._find_workspace_root()
            self.assertEqual(root, workspace_dir)

    def test_find_root_with_defaults_file_relative_path(self):
        workspace_dir = os.path.join(self.test_dir, "relative", "workspace")
        os.makedirs(workspace_dir)
        defaults_path = os.path.join(self.test_dir, "defaults.yaml")
        with open(defaults_path, "w", encoding="utf-8") as f:
            f.write("base-path: ./relative/workspace\n")

        with mock.patch.dict(os.environ, {"COLCON_DEFAULTS_FILE": defaults_path}):
            root = colcon_runner._find_workspace_root()
            self.assertEqual(root, workspace_dir)

    def test_find_root_with_base_paths_list(self):
        defaults_path = os.path.join(self.test_dir, "defaults.yaml")
        valid_workspace = os.path.join(self.test_dir, "workspace_list", "ws")
        os.makedirs(valid_workspace)
        with open(defaults_path, "w", encoding="utf-8") as f:
            f.write("build:\n  base-paths:\n    - ./nonexistent\n    - ./workspace_list/ws\n")

        with mock.patch.dict(os.environ, {"COLCON_DEFAULTS_FILE": defaults_path}):
            root = colcon_runner._find_workspace_root()
            self.assertEqual(root, os.path.abspath(valid_workspace))

    def test_find_root_fallback_to_src(self):
        # Create src directory
        src_dir = os.path.join(self.test_dir, "src")
        os.mkdir(src_dir)
        self.addCleanup(os.rmdir, src_dir)

        # Ensure no defaults file env var
        with mock.patch.dict(os.environ, {}, clear=True):
            cwd = os.getcwd()
            try:
                os.chdir(self.test_dir)
                root = colcon_runner._find_workspace_root()
                self.assertEqual(root, self.test_dir)
            finally:
                os.chdir(cwd)

    def test_find_root_defaults_file_missing_base_path(self):
        defaults_path = os.path.join(self.test_dir, "defaults.yaml")
        with open(defaults_path, "w", encoding="utf-8") as f:
            f.write("other-key: value\n")
        # Create src directory for fallback
        src_dir = os.path.join(self.test_dir, "src")
        os.mkdir(src_dir)
        self.addCleanup(os.rmdir, src_dir)
        with mock.patch.dict(os.environ, {"COLCON_DEFAULTS_FILE": defaults_path}, clear=True):
            cwd = os.getcwd()
            try:
                os.chdir(self.test_dir)
                root = colcon_runner._find_workspace_root()
                self.assertEqual(root, self.test_dir)
            finally:
                os.chdir(cwd)

    def test_find_root_with_colcon_home(self):
        # Create defaults file in custom COLCON_HOME
        colcon_home = os.path.join(self.test_dir, "custom_home")
        os.makedirs(colcon_home)
        defaults_path = os.path.join(colcon_home, "defaults.yaml")
        workspace_dir = os.path.join(self.test_dir, "home_workspace")
        os.makedirs(workspace_dir)
        with open(defaults_path, "w", encoding="utf-8") as f:
            f.write(f"base-path: {workspace_dir}\n")
        with mock.patch.dict(os.environ, {"COLCON_HOME": colcon_home}, clear=True):
            root = colcon_runner._find_workspace_root()
            self.assertEqual(root, workspace_dir)

    def test_find_root_default_location(self):
        # Mock os.path.expanduser to point to our test dir
        fake_home = os.path.join(self.test_dir, "home")
        fake_colcon_dir = os.path.join(fake_home, ".colcon")
        os.makedirs(fake_colcon_dir)
        defaults_path = os.path.join(fake_colcon_dir, "defaults.yaml")
        workspace_dir = os.path.join(self.test_dir, "default_workspace")
        os.makedirs(workspace_dir)
        with open(defaults_path, "w", encoding="utf-8") as f:
            f.write(f"base-path: {workspace_dir}\n")
        with mock.patch("os.path.expanduser", return_value=defaults_path):
            with mock.patch.dict(os.environ, {}, clear=True):
                root = colcon_runner._find_workspace_root()
                self.assertEqual(root, workspace_dir)

    def test_precedence_env_over_home(self):
        # Setup both env var and COLCON_HOME
        # 1. Env var file
        env_file = os.path.join(self.test_dir, "env_defaults.yaml")
        env_workspace = os.path.join(self.test_dir, "env_workspace")
        os.makedirs(env_workspace)
        with open(env_file, "w", encoding="utf-8") as f:
            f.write(f"base-path: {env_workspace}\n")
        # 2. COLCON_HOME file
        colcon_home = os.path.join(self.test_dir, "colcon_home")
        os.makedirs(colcon_home)
        home_file = os.path.join(colcon_home, "defaults.yaml")
        home_workspace = os.path.join(self.test_dir, "home_workspace")
        os.makedirs(home_workspace)
        with open(home_file, "w", encoding="utf-8") as f:
            f.write(f"base-path: {home_workspace}\n")
        with mock.patch.dict(
            os.environ, {"COLCON_DEFAULTS_FILE": env_file, "COLCON_HOME": colcon_home}
        ):
            root = colcon_runner._find_workspace_root()
            self.assertEqual(root, env_workspace)


class VersionTests(unittest.TestCase):
    def test_version_flag(self):
        # Test --version flag
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            with self.assertRaises(SystemExit) as cm:
                colcon_runner.main(["--version"])

        self.assertEqual(cm.exception.code, 0)
        output = buf.getvalue()
        self.assertIn("cr (colcon-runner) version", output)
        # Assert it contains the actual package version
        expected_version = version("colcon-runner")
        self.assertIn(expected_version, output)

    def test_version_short_flag(self):
        # Test -v flag
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            with self.assertRaises(SystemExit) as cm:
                colcon_runner.main(["-v"])

        self.assertEqual(cm.exception.code, 0)
        output = buf.getvalue()
        self.assertIn("cr (colcon-runner) version", output)
        # Assert it contains the actual package version
        expected_version = version("colcon-runner")
        self.assertIn(expected_version, output)

    def test_version_not_installed(self):
        # Test fallback path when the package is not installed
        from importlib.metadata import PackageNotFoundError

        buf = io.StringIO()
        # Mock the version function to raise PackageNotFoundError
        with mock.patch("colcon_runner.colcon_runner.version") as mock_version:
            mock_version.side_effect = PackageNotFoundError

            with contextlib.redirect_stdout(buf):
                with self.assertRaises(SystemExit) as cm:
                    colcon_runner.main(["--version"])

        self.assertEqual(cm.exception.code, 0)
        output = buf.getvalue()
        self.assertIn("cr (colcon-runner) version unknown (not installed)", output)


class ErrorHandlingTests(unittest.TestCase):
    """Test that CLI errors are user-friendly without stack traces."""

    def _run_main(self, argv):
        """Helper to run main() capturing stdout/stderr and SystemExit."""
        buf_out = io.StringIO()
        buf_err = io.StringIO()
        with contextlib.redirect_stdout(buf_out), contextlib.redirect_stderr(buf_err):
            with self.assertRaises(SystemExit) as cm:
                colcon_runner.main(argv)
        return cm.exception.code, buf_out.getvalue(), buf_err.getvalue()

    def _assert_no_stack_trace(self, stderr: str):
        """Ensure no Python traceback is shown to the user."""
        self.assertNotIn("Traceback (most recent call last)", stderr)
        # Don't check for "Exception:" or "KeyboardInterrupt" as these might appear in error messages
        self.assertNotIn("ParseError", stderr)
        self.assertNotIn('File "', stderr)

    def test_unknown_command_no_stack_trace(self):
        """Test that unknown commands show clean error without stack trace."""
        exit_code, _stdout, stderr = self._run_main(["h"])

        # Should exit with code 1
        self.assertEqual(exit_code, 1)

        # Check error output
        self.assertIn("Error: unknown command letter 'h'", stderr)
        self.assertIn(
            "Valid commands: s (set), b (build), t (test), c (clean), i (install)", stderr
        )
        self.assertIn("Use 'cr --help' for more information", stderr)

        # Ensure no stack trace is present
        self._assert_no_stack_trace(stderr)

    def test_multiple_unknown_commands_no_stack_trace(self):
        """Test that multiple invalid commands also show clean errors."""
        exit_code, _stdout, stderr = self._run_main(["xyz"])

        self.assertEqual(exit_code, 1)
        self.assertIn("Error: unknown command letter 'x'", stderr)

        # Assert full friendly help text as in single command test
        self.assertIn(
            "Valid commands: s (set), b (build), t (test), c (clean), i (install)", stderr
        )
        self.assertIn("Use 'cr --help' for more information", stderr)

        self._assert_no_stack_trace(stderr)

    def test_invalid_package_name_no_stack_trace(self):
        """Test that invalid package names show clean errors."""
        exit_code, _stdout, stderr = self._run_main(["bo", "../etc/passwd", "--dry-run"])

        self.assertEqual(exit_code, 1)
        self.assertIn("Error:", stderr)
        self.assertIn("path traversal", stderr)

        # Ensure raw unsafe path is not echoed back to user
        self.assertNotIn("../etc/passwd", stderr)

        self._assert_no_stack_trace(stderr)

    def test_missing_package_no_stack_trace(self):
        """Test that missing package argument shows clean error."""
        # Remove default package if exists
        if os.path.exists(colcon_runner.PKG_FILE):
            backup = colcon_runner.PKG_FILE + ".test_backup"
            shutil.copy(colcon_runner.PKG_FILE, backup)
            os.remove(colcon_runner.PKG_FILE)
            self.addCleanup(
                lambda: os.path.exists(backup) and shutil.move(backup, colcon_runner.PKG_FILE)
            )

        exit_code, _stdout, stderr = self._run_main(["bo"])

        self.assertEqual(exit_code, 1)
        self.assertIn("Error: no package specified and no default set", stderr)
        self._assert_no_stack_trace(stderr)

    def test_workspace_not_found_no_stack_trace(self):
        """Test that missing workspace shows clean error."""
        # Create a temp dir without src directory
        with tempfile.TemporaryDirectory() as tmpdir:
            # Clear environment variables
            with mock.patch.dict(os.environ, {}, clear=True):
                cwd = os.getcwd()
                try:
                    os.chdir(tmpdir)
                    exit_code, _stdout, stderr = self._run_main(["ia", "--dry-run"])
                finally:
                    os.chdir(cwd)

        self.assertEqual(exit_code, 1)
        self.assertIn("Error:", stderr)
        self.assertIn("Could not find workspace root", stderr)
        self._assert_no_stack_trace(stderr)

    def test_keyboard_interrupt_handling(self):
        """KeyboardInterrupt should exit with 130 and a friendly message, no stack trace."""

        def raise_keyboard_interrupt(*_args, **_kwargs):
            raise KeyboardInterrupt()

        with mock.patch(
            "colcon_runner.colcon_runner._parse_verbs",
            side_effect=raise_keyboard_interrupt,
        ):
            exit_code, _stdout, stderr = self._run_main(["b"])

        self.assertEqual(exit_code, 130)
        self.assertIn("Interrupted by user", stderr)
        self._assert_no_stack_trace(stderr)

    def test_file_io_error_handling(self):
        """File/permission errors should show the OS error message but no stack trace."""

        def raise_permission_error(*_args, **_kwargs):
            raise PermissionError("Permission denied")

        # Mock open() used by save_default_pkg to simulate a file error
        with mock.patch("builtins.open", side_effect=raise_permission_error):
            exit_code, _stdout, stderr = self._run_main(["s", "demo_pkg"])

        self.assertEqual(exit_code, 1)
        self.assertIn("Error: OS error while running colcon-runner:", stderr)
        self.assertIn("Permission denied", stderr)
        # No stack trace for OS errors
        self._assert_no_stack_trace(stderr)

    def test_unexpected_exception_handling(self):
        """Generic unexpected exceptions should be caught and reported with traceback for debugging."""

        def raise_generic_exception(*_args, **_kwargs):
            raise RuntimeError("simulated internal error for testing")

        with mock.patch(
            "colcon_runner.colcon_runner._parse_verbs",
            side_effect=raise_generic_exception,
        ):
            exit_code, _stdout, stderr = self._run_main(["b"])

        self.assertEqual(exit_code, 1)
        self.assertIn("Error: unexpected error:", stderr)

        # For unexpected exceptions, we DO want the traceback logged to stderr for debugging
        self.assertIn("Traceback (most recent call last)", stderr)
        self.assertIn("RuntimeError", stderr)


class ShellIntegrationTests(unittest.TestCase):
    """Test shell integration functionality (literal bash pattern)."""

    def test_install_shell_integration_fresh(self):
        """Test --install-shell-integration on a fresh bashrc."""
        with tempfile.TemporaryDirectory() as temp_dir:
            bashrc_path = os.path.join(temp_dir, ".bashrc")

            with mock.patch.dict(os.environ, {"HOME": temp_dir}):
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    with self.assertRaises(SystemExit) as cm:
                        colcon_runner.main(["--install-shell-integration"])

                self.assertEqual(cm.exception.code, 0)
                output = buf.getvalue()

                self.assertIn("Shell integration installed to ~/.bashrc", output)
                self.assertIn("source ~/.bashrc", output)

                self.assertTrue(os.path.exists(bashrc_path))
                with open(bashrc_path, "r", encoding="utf-8") as f:
                    content = f.read()
                    self.assertIn("# Colcon-runner shell integration v", content)
                    self.assertIn("cr() {", content)
                    self.assertIn("_cr_completions()", content)
                    self.assertIn("complete -F _cr_completions cr", content)
                    self.assertIn("--workspace-root", content)

    def test_install_shell_integration_idempotent(self):
        """Test that installing shell integration twice is idempotent."""
        with tempfile.TemporaryDirectory() as temp_dir:
            bashrc_path = os.path.join(temp_dir, ".bashrc")

            with mock.patch.dict(os.environ, {"HOME": temp_dir}):
                # First installation
                buf1 = io.StringIO()
                with contextlib.redirect_stdout(buf1):
                    with self.assertRaises(SystemExit) as cm:
                        colcon_runner.main(["--install-shell-integration"])
                self.assertEqual(cm.exception.code, 0)

                # Second installation
                buf2 = io.StringIO()
                with contextlib.redirect_stdout(buf2):
                    with self.assertRaises(SystemExit) as cm:
                        colcon_runner.main(["--install-shell-integration"])
                self.assertEqual(cm.exception.code, 0)

                output2 = buf2.getvalue()
                self.assertIn("Shell integration is already installed", output2)

                # Verify only one integration block exists
                with open(bashrc_path, "r", encoding="utf-8") as f:
                    content = f.read()
                    self.assertEqual(content.count("# Colcon-runner shell integration v"), 1)
                    self.assertEqual(content.count("complete -F _cr_completions cr"), 1)

    def test_install_shell_integration_preserves_existing_bashrc(self):
        """Test that existing bashrc content is preserved."""
        with tempfile.TemporaryDirectory() as temp_dir:
            bashrc_path = os.path.join(temp_dir, ".bashrc")

            existing_content = "# My custom bashrc\nexport MY_VAR=123\nalias ll='ls -l'\n"
            with open(bashrc_path, "w", encoding="utf-8") as f:
                f.write(existing_content)

            with mock.patch.dict(os.environ, {"HOME": temp_dir}):
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    with self.assertRaises(SystemExit) as cm:
                        colcon_runner.main(["--install-shell-integration"])

                self.assertEqual(cm.exception.code, 0)

                with open(bashrc_path, "r", encoding="utf-8") as f:
                    content = f.read()
                    self.assertIn("export MY_VAR=123", content)
                    self.assertIn("alias ll='ls -l'", content)
                    self.assertIn("# Colcon-runner shell integration v", content)
                    self.assertIn("complete -F _cr_completions cr", content)

    def test_install_shell_integration_creates_bashrc_if_missing(self):
        """Test that ~/.bashrc is created if it doesn't exist."""
        with tempfile.TemporaryDirectory() as temp_dir:
            bashrc_path = os.path.join(temp_dir, ".bashrc")

            self.assertFalse(os.path.exists(bashrc_path))

            with mock.patch.dict(os.environ, {"HOME": temp_dir}):
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    with self.assertRaises(SystemExit) as cm:
                        colcon_runner.main(["--install-shell-integration"])

                self.assertEqual(cm.exception.code, 0)
                output = buf.getvalue()
                self.assertIn(f"Creating {bashrc_path}", output)

                self.assertTrue(os.path.exists(bashrc_path))
                with open(bashrc_path, "r", encoding="utf-8") as f:
                    content = f.read()
                    self.assertIn("# .bashrc", content)
                    self.assertIn("# Colcon-runner shell integration v", content)

    def test_install_replaces_old_version(self):
        """Test that --install-shell-integration replaces a previous version block."""
        with tempfile.TemporaryDirectory() as temp_dir:
            bashrc_path = os.path.join(temp_dir, ".bashrc")

            old_bashrc = (
                "# My config\nexport FOO=bar\n\n"
                "# Colcon-runner shell integration v0.9.0 (generated by cr --init-bash)\n"
                "cr() {\n"
                '    command cr "$@"\n'
                "}\n"
                "_cr_completions() {\n"
                "    COMPREPLY=()\n"
                "}\n"
                "complete -F _cr_completions cr\n"
            )
            with open(bashrc_path, "w", encoding="utf-8") as f:
                f.write(old_bashrc)

            with mock.patch.dict(os.environ, {"HOME": temp_dir}):
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    with self.assertRaises(SystemExit) as cm:
                        colcon_runner.main(["--install-shell-integration"])

                self.assertEqual(cm.exception.code, 0)
                output = buf.getvalue()
                self.assertIn("Updated", output)

                with open(bashrc_path, "r", encoding="utf-8") as f:
                    content = f.read()
                    self.assertIn("export FOO=bar", content)
                    # Should have new version, not old
                    self.assertNotIn("v0.9.0", content)
                    self.assertIn("# Colcon-runner shell integration v", content)
                    self.assertIn("--workspace-root", content)
                    self.assertEqual(content.count("# Colcon-runner shell integration v"), 1)


class PackageListingTests(unittest.TestCase):
    """Test package listing functionality for shell completion."""

    def setUp(self):
        # Create temp workspace with package.xml files
        self.test_dir = tempfile.mkdtemp()
        self.src_dir = os.path.join(self.test_dir, "src")
        os.makedirs(self.src_dir)

        for pkg_name in ["pkg_a", "pkg_b", "pkg_c"]:
            pkg_dir = os.path.join(self.src_dir, pkg_name)
            os.makedirs(pkg_dir)
            with open(os.path.join(pkg_dir, "package.xml"), "w", encoding="utf-8") as f:
                f.write(f'<?xml version="1.0"?>\n<package><name>{pkg_name}</name></package>')

    def tearDown(self):
        shutil.rmtree(self.test_dir)

    def test_list_packages_finds_all(self):
        """Test that _list_packages finds all packages in workspace."""
        # pylint: disable=protected-access
        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=self.test_dir):
            packages = colcon_runner._list_packages()
            self.assertEqual(packages, ["pkg_a", "pkg_b", "pkg_c"])

    def test_list_packages_handles_nested_packages(self):
        """Test packages in nested directories (e.g., src/group/pkg)."""
        # pylint: disable=protected-access
        # Create a nested package
        nested_dir = os.path.join(self.src_dir, "group", "nested_pkg")
        os.makedirs(nested_dir)
        with open(os.path.join(nested_dir, "package.xml"), "w", encoding="utf-8") as f:
            f.write('<?xml version="1.0"?>\n<package><name>nested_pkg</name></package>')

        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=self.test_dir):
            packages = colcon_runner._list_packages()
            self.assertIn("nested_pkg", packages)
            self.assertEqual(len(packages), 4)  # pkg_a, pkg_b, pkg_c, nested_pkg

    def test_list_packages_handles_malformed_xml(self):
        """Test that malformed package.xml files are skipped without error."""
        # pylint: disable=protected-access
        # Create a malformed package.xml
        bad_pkg_dir = os.path.join(self.src_dir, "bad_pkg")
        os.makedirs(bad_pkg_dir)
        with open(os.path.join(bad_pkg_dir, "package.xml"), "w", encoding="utf-8") as f:
            f.write("not valid xml <<<<")

        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=self.test_dir):
            packages = colcon_runner._list_packages()
            # Should still find the valid packages
            self.assertEqual(packages, ["pkg_a", "pkg_b", "pkg_c"])

    def test_list_packages_handles_missing_name_element(self):
        """Test that package.xml without <name> is skipped."""
        # pylint: disable=protected-access
        # Create a package.xml without name element
        no_name_dir = os.path.join(self.src_dir, "no_name_pkg")
        os.makedirs(no_name_dir)
        with open(os.path.join(no_name_dir, "package.xml"), "w", encoding="utf-8") as f:
            f.write('<?xml version="1.0"?>\n<package><version>1.0</version></package>')

        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=self.test_dir):
            packages = colcon_runner._list_packages()
            # Should still find the valid packages
            self.assertEqual(packages, ["pkg_a", "pkg_b", "pkg_c"])

    def test_list_packages_quiet_on_no_workspace(self):
        """Test that _list_packages returns [] when workspace not found."""
        # pylint: disable=protected-access
        with mock.patch.object(
            colcon_runner,
            "_find_workspace_root",
            side_effect=colcon_runner.ParseError("No workspace"),
        ):
            packages = colcon_runner._list_packages()
            self.assertEqual(packages, [])

    def test_list_packages_deduplicates(self):
        """Test that same package name in multiple locations appears once."""
        # pylint: disable=protected-access
        # Create duplicate package name in different location
        dup_dir = os.path.join(self.src_dir, "another", "pkg_a")
        os.makedirs(dup_dir)
        with open(os.path.join(dup_dir, "package.xml"), "w", encoding="utf-8") as f:
            f.write('<?xml version="1.0"?>\n<package><name>pkg_a</name></package>')

        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=self.test_dir):
            packages = colcon_runner._list_packages()
            # pkg_a should appear only once
            self.assertEqual(packages.count("pkg_a"), 1)
            self.assertEqual(len(packages), 3)

    def test_list_packages_skips_build_dirs(self):
        """Test that _list_packages skips packages in build/install/log/test_result."""
        # pylint: disable=protected-access
        for skip_dir in ("build", "install", "log", "test_result"):
            pkg_dir = os.path.join(self.test_dir, skip_dir, f"pkg_in_{skip_dir}")
            os.makedirs(pkg_dir)
            with open(os.path.join(pkg_dir, "package.xml"), "w", encoding="utf-8") as f:
                f.write(f'<?xml version="1.0"?>\n<package><name>pkg_in_{skip_dir}</name></package>')

        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=self.test_dir):
            packages = colcon_runner._list_packages()

        for skip_dir in ("build", "install", "log", "test_result"):
            self.assertNotIn(f"pkg_in_{skip_dir}", packages)
        self.assertEqual(len(packages), 3)

    def test_list_packages_outside_src(self):
        """Test that packages outside src/ are found when workspace root has no src/."""
        # pylint: disable=protected-access
        workspace = tempfile.mkdtemp()
        self.addCleanup(lambda: shutil.rmtree(workspace))

        pkg_dir = os.path.join(workspace, "my_pkg")
        os.makedirs(pkg_dir)
        with open(os.path.join(pkg_dir, "package.xml"), "w", encoding="utf-8") as f:
            f.write('<?xml version="1.0"?>\n<package><name>my_pkg</name></package>')

        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=workspace):
            packages = colcon_runner._list_packages()
            self.assertIn("my_pkg", packages)

    def test_list_packages_no_src_directory(self):
        """Test that _list_packages returns [] when no src directory exists."""
        # pylint: disable=protected-access
        # Create a workspace without src dir
        empty_workspace = tempfile.mkdtemp()
        self.addCleanup(lambda: shutil.rmtree(empty_workspace))

        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=empty_workspace):
            packages = colcon_runner._list_packages()
            self.assertEqual(packages, [])

    def test_list_packages_cli_flag(self):
        """Test that `cr --list-packages` outputs package names."""
        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=self.test_dir):
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                with self.assertRaises(SystemExit) as cm:
                    colcon_runner.main(["--list-packages"])

            self.assertEqual(cm.exception.code, 0)
            output = buf.getvalue()
            # Check output contains package names, one per line
            self.assertIn("pkg_a\n", output)
            self.assertIn("pkg_b\n", output)
            self.assertIn("pkg_c\n", output)

    def test_list_packages_handles_whitespace_in_name(self):
        """Test that package names with whitespace are trimmed."""
        # pylint: disable=protected-access
        # Create a package with whitespace in name element
        ws_dir = os.path.join(self.src_dir, "whitespace_pkg")
        os.makedirs(ws_dir)
        with open(os.path.join(ws_dir, "package.xml"), "w", encoding="utf-8") as f:
            f.write('<?xml version="1.0"?>\n<package><name>  whitespace_pkg  </name></package>')

        with mock.patch.object(colcon_runner, "_find_workspace_root", return_value=self.test_dir):
            packages = colcon_runner._list_packages()
            self.assertIn("whitespace_pkg", packages)
            # Ensure no whitespace version exists
            self.assertNotIn("  whitespace_pkg  ", packages)


class InitBashTests(unittest.TestCase):
    """Test _get_init_bash() output and --init-bash flag."""

    def test_init_bash_contains_completion(self):
        """Test that _get_init_bash returns the completion function."""
        # pylint: disable=protected-access
        script = colcon_runner._get_init_bash()

        self.assertIn("_cr_completions()", script)
        self.assertIn("--list-packages", script)
        self.assertIn("complete -F _cr_completions cr", script)
        self.assertIn("COMP_CWORD -eq 1", script)
        self.assertIn("COMP_CWORD -eq 2", script)
        # Should skip completion when typing an option (--something)
        self.assertIn("-*", script)

    def test_init_bash_contains_cr_wrapper(self):
        """Test that _get_init_bash includes the cr() wrapper with targeted sourcing."""
        # pylint: disable=protected-access
        script = colcon_runner._get_init_bash()

        self.assertIn("cr() {", script)
        self.assertIn("command cr", script)
        self.assertIn("--install-base", script)
        self.assertIn("install/local_setup.bash", script)
        self.assertIn("install/setup.bash", script)
        # Must not re-source full bashrc
        self.assertNotIn('source "$HOME/.bashrc"', script)

    def test_init_bash_contains_version_comment(self):
        """Test that _get_init_bash output starts with a version comment."""
        # pylint: disable=protected-access
        script = colcon_runner._get_init_bash()

        self.assertTrue(
            script.startswith("# Colcon-runner shell integration v"),
            f"Script should start with version comment, got: {script[:60]}",
        )
        self.assertIn("(generated by cr --init-bash)", script)

    def test_init_bash_flag_outputs_script(self):
        """Test that `cr --init-bash` prints the full script and exits 0."""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            with self.assertRaises(SystemExit) as cm:
                colcon_runner.main(["--init-bash"])

        self.assertEqual(cm.exception.code, 0)
        output = buf.getvalue()
        self.assertIn("cr() {", output)
        self.assertIn("_cr_completions()", output)
        self.assertIn("complete -F _cr_completions cr", output)


class WorkspaceRootFlagTests(unittest.TestCase):
    """Test --workspace-root hidden flag."""

    def test_workspace_root_outputs_path(self):
        """Test that --workspace-root prints the workspace root and exits 0."""
        with mock.patch.object(
            colcon_runner, "_find_workspace_root", return_value="/fake/workspace"
        ):
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                with self.assertRaises(SystemExit) as cm:
                    colcon_runner.main(["--workspace-root"])

            self.assertEqual(cm.exception.code, 0)
            self.assertEqual(buf.getvalue().strip(), "/fake/workspace")

    def test_workspace_root_exits_1_on_error(self):
        """Test that --workspace-root exits 1 when no workspace is found."""
        with mock.patch.object(
            colcon_runner,
            "_find_workspace_root",
            side_effect=colcon_runner.ParseError("no workspace"),
        ):
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                with self.assertRaises(SystemExit) as cm:
                    colcon_runner.main(["--workspace-root"])

            self.assertEqual(cm.exception.code, 1)
            self.assertEqual(buf.getvalue().strip(), "")


class RemoveShellIntegrationTests(unittest.TestCase):
    """Test _remove_shell_integration() block removal."""

    # pylint: disable=protected-access

    def test_removes_literal_integration_block(self):
        content = (
            "# before\n"
            "# Colcon-runner shell integration v0.10.0 (generated by cr --init-bash)\n"
            "cr() {\n"
            '    command cr "$@"\n'
            "}\n"
            "_cr_completions() {\n"
            "    COMPREPLY=()\n"
            "}\n"
            "complete -F _cr_completions cr\n"
            "# after\n"
        )
        cleaned, modified = colcon_runner._remove_shell_integration(content)
        self.assertTrue(modified)
        self.assertIn("# before", cleaned)
        self.assertIn("# after", cleaned)
        self.assertNotIn("cr()", cleaned)
        self.assertNotIn("_cr_completions", cleaned)

    def test_no_blocks_returns_unmodified(self):
        content = "# My bashrc\nexport PATH=/usr/bin\n"
        cleaned, modified = colcon_runner._remove_shell_integration(content)
        self.assertFalse(modified)
        self.assertEqual(cleaned, content)

    def test_preserves_surrounding_content(self):
        content = (
            "alias ls='ls --color'\n"
            "# Colcon-runner shell integration v0.10.0 (generated by cr --init-bash)\n"
            "cr() {\n"
            "    COMPREPLY=()\n"
            "}\n"
            "complete -F _cr_completions cr\n"
            "alias grep='grep --color'\n"
        )
        cleaned, modified = colcon_runner._remove_shell_integration(content)
        self.assertTrue(modified)
        self.assertIn("alias ls='ls --color'", cleaned)
        self.assertIn("alias grep='grep --color'", cleaned)


class ParseVerbsDefaultSpecTests(unittest.TestCase):
    """Test _parse_verbs with non-default default_spec parameter."""

    # pylint: disable=protected-access

    def test_default_spec_u(self):
        # When default_spec="u", unspecified verbs default to "u"
        self.assertEqual(colcon_runner._parse_verbs("b", default_spec="u"), [("b", "u")])
        self.assertEqual(colcon_runner._parse_verbs("t", default_spec="u"), [("t", "u")])
        self.assertEqual(colcon_runner._parse_verbs("c", default_spec="u"), [("c", "u")])
        self.assertEqual(colcon_runner._parse_verbs("i", default_spec="u"), [("i", "u")])

    def test_default_spec_u_compound(self):
        self.assertEqual(
            colcon_runner._parse_verbs("bt", default_spec="u"), [("b", "u"), ("t", "u")]
        )
        self.assertEqual(
            colcon_runner._parse_verbs("cbt", default_spec="u"),
            [("c", "u"), ("b", "u"), ("t", "u")],
        )

    def test_explicit_spec_overrides_default(self):
        # Explicit specifiers override default_spec
        self.assertEqual(colcon_runner._parse_verbs("bo", default_spec="u"), [("b", "o")])
        self.assertEqual(colcon_runner._parse_verbs("ba", default_spec="u"), [("b", "a")])
        self.assertEqual(colcon_runner._parse_verbs("bu", default_spec="u"), [("b", "u")])

    def test_mixed_explicit_and_default(self):
        # "cabuto" with default_spec="u": ca=explicit, bu=explicit, to=explicit
        self.assertEqual(
            colcon_runner._parse_verbs("cabuto", default_spec="u"),
            [("c", "a"), ("b", "u"), ("t", "o")],
        )
        # "boto" with default_spec="u": bo=explicit, to=explicit
        self.assertEqual(
            colcon_runner._parse_verbs("boto", default_spec="u"), [("b", "o"), ("t", "o")]
        )

    def test_set_verb_unaffected_by_default_spec(self):
        # 's' verb has no specifier regardless of default_spec
        self.assertEqual(colcon_runner._parse_verbs("s", default_spec="u"), [("s", None)])


class PackageFirstIntegrationTests(unittest.TestCase):
    """Test package-first argument order: cr PKG [VERB]."""

    def setUp(self):
        # Mock workspace root detection
        self.workspace_patch = mock.patch.object(
            colcon_runner, "_find_workspace_root", return_value="/test/workspace"
        )
        self.workspace_patch.start()
        self.addCleanup(self.workspace_patch.stop)

        # Mock _list_packages to return known packages
        self.list_patch = mock.patch.object(
            colcon_runner, "_list_packages", return_value=["my_pkg", "other_pkg", "pkg-with-dash"]
        )
        self.list_patch.start()
        self.addCleanup(self.list_patch.stop)

    def test_pkg_only_defaults_to_build_upto(self):
        """cr my_pkg -> build --packages-up-to my_pkg."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon build --packages-up-to my_pkg", output)

    def test_pkg_with_bo(self):
        """cr my_pkg bo -> build --packages-select my_pkg."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "bo", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon build --packages-select my_pkg", output)

    def test_pkg_with_bu(self):
        """cr my_pkg bu -> build --packages-up-to my_pkg."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "bu", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon build --packages-up-to my_pkg", output)

    def test_pkg_with_bt_compound(self):
        """cr my_pkg bt -> build upto + test upto."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "bt", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon build --packages-up-to my_pkg", output)
            self.assertIn("colcon test --packages-up-to my_pkg", output)

    def test_pkg_with_boto_compound(self):
        """cr my_pkg boto -> build only + test only."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "boto", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon build --packages-select my_pkg", output)
            self.assertIn("colcon test --packages-select my_pkg", output)

    def test_pkg_with_cbt_compound(self):
        """cr my_pkg cbt -> clean upto + build upto + test upto."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "cbt", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon clean packages", output)
            self.assertIn("--packages-up-to my_pkg", output)
            self.assertIn("colcon build --packages-up-to my_pkg", output)
            self.assertIn("colcon test --packages-up-to my_pkg", output)

    def test_pkg_with_ca_overrides_default(self):
        """cr my_pkg ca -> clean all (explicit 'a' overrides default 'u')."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "ca", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon clean workspace", output)

    def test_pkg_with_set_verb(self):
        """cr my_pkg s -> set default package."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "s"])
            output = buf.getvalue()
            self.assertIn("Default package set to 'my_pkg'", output)

    def test_pkg_with_dash_in_name(self):
        """cr pkg-with-dash -> build --packages-up-to pkg-with-dash."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["pkg-with-dash", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon build --packages-up-to pkg-with-dash", output)

    def test_pkg_with_install_verb(self):
        """cr my_pkg i -> rosdep install upto."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "i", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("rosdep install --from-paths /test/workspace/my_pkg", output)

    def test_pkg_with_cabuto_compound(self):
        """cr my_pkg cabuto -> clean all, build upto, test only."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["my_pkg", "cabuto", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon clean workspace", output)
            self.assertIn("colcon build --packages-up-to my_pkg", output)
            self.assertIn("colcon test --packages-select my_pkg", output)


class VerbFirstBackwardCompatTests(unittest.TestCase):
    """Test that existing verb-first commands still work unchanged."""

    def setUp(self):
        # Mock workspace root detection
        self.workspace_patch = mock.patch.object(
            colcon_runner, "_find_workspace_root", return_value="/test/workspace"
        )
        self.workspace_patch.start()
        self.addCleanup(self.workspace_patch.stop)

        # Mock _list_packages to return known packages (none matching verbs)
        self.list_patch = mock.patch.object(
            colcon_runner, "_list_packages", return_value=["my_pkg", "other_pkg"]
        )
        self.list_patch.start()
        self.addCleanup(self.list_patch.stop)

    def test_verb_first_build_all(self):
        """cr b -> build all (verb-first mode)."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["b", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon build --dry-run", output)
            # Should NOT contain --packages-up-to or --packages-select
            self.assertNotIn("--packages-up-to", output)
            self.assertNotIn("--packages-select", output)

    def test_verb_first_bo_with_pkg(self):
        """cr bo my_pkg -> build only my_pkg (verb-first mode)."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["bo", "my_pkg", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon build --packages-select my_pkg", output)

    def test_verb_first_cbt(self):
        """cr cbt -> clean all, build all, test all (verb-first mode)."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["cbt", "--dry-run"])
            output = buf.getvalue()
            self.assertIn("colcon clean workspace", output)
            self.assertIn("colcon build", output)
            self.assertIn("colcon test", output)

    def test_unknown_first_arg_errors(self):
        """cr xyz -> error (not a package, not a valid verb)."""
        buf_err = io.StringIO()
        with contextlib.redirect_stderr(buf_err):
            with self.assertRaises(SystemExit) as cm:
                colcon_runner.main(["xyz"])
        self.assertEqual(cm.exception.code, 1)
        self.assertIn("unknown command letter 'x'", buf_err.getvalue())

    def test_no_warning_in_package_first_mode(self):
        """Package-first mode should not trigger the 'defaulted to all' warning."""
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0
            # my_pkg ca -> clean all; no warning since it's package-first mode
            with mock.patch.object(colcon_runner.logger, "warning") as mock_warn:
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    colcon_runner.main(["my_pkg", "ca", "--dry-run"])
                mock_warn.assert_not_called()
            output = buf.getvalue()
            self.assertIn("colcon clean workspace", output)

    def test_verb_like_package_name_stays_verb_first(self):
        """A package named 'b' should not shadow the 'b' verb."""
        # _list_packages returns a package named "b"
        self.list_patch.stop()
        with mock.patch.object(colcon_runner, "_list_packages", return_value=["b", "my_pkg"]):
            with mock.patch.object(colcon_runner, "subprocess") as m_sp:
                m_sp.run.return_value.returncode = 0
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    colcon_runner.main(["b", "--dry-run"])
                output = buf.getvalue()
                # "b" should be parsed as verb (build all), not as package
                self.assertIn("colcon build --dry-run", output)
                self.assertNotIn("--packages-up-to", output)
        self.list_patch.start()


class PackageFirstListPackagesFailureTests(unittest.TestCase):
    """Test that package-first detection gracefully handles _list_packages failures."""

    def setUp(self):
        self.workspace_patch = mock.patch.object(
            colcon_runner, "_find_workspace_root", return_value="/test/workspace"
        )
        self.workspace_patch.start()
        self.addCleanup(self.workspace_patch.stop)

    def test_list_packages_exception_falls_back_to_verb_first(self):
        """When _list_packages raises, fall back to verb-first mode."""
        with mock.patch.object(colcon_runner, "_list_packages", side_effect=OSError("boom")):
            with mock.patch.object(colcon_runner, "subprocess") as m_sp:
                m_sp.run.return_value.returncode = 0
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    colcon_runner.main(["b", "--dry-run"])
                output = buf.getvalue()
                self.assertIn("colcon build --dry-run", output)


if __name__ == "__main__":  # pragma: no cover — run the tests
    unittest.main(verbosity=2)
