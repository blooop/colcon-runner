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


class RosdepCacheTests(unittest.TestCase):
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
        cache_file = colcon_runner._get_rosdep_cache_file()
        self.assertIn("/tmp/colcon_runner_rosdep_update_", cache_file)
        # Should end with date in YYYY-MM-DD format
        import re

        self.assertTrue(re.search(r"\d{4}-\d{2}-\d{2}$", cache_file))

    def test_rosdep_update_needed_no_cache(self):
        # When cache file doesn't exist, update is needed
        with mock.patch.object(colcon_runner.os.path, "exists", return_value=False):
            self.assertTrue(colcon_runner._rosdep_update_needed())

    def test_rosdep_update_not_needed_with_cache(self):
        # When cache file exists, update is not needed
        with mock.patch.object(colcon_runner.os.path, "exists", return_value=True):
            self.assertFalse(colcon_runner._rosdep_update_needed())

    def test_rosdep_update_runs_once_then_skipped(self):
        # Test that rosdep update runs first time, then is skipped
        with mock.patch.object(colcon_runner, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            with mock.patch.object(colcon_runner, "_rosdep_update_needed") as m_update_needed:
                with mock.patch.object(colcon_runner, "_mark_rosdep_updated"):
                    # First call - update is needed
                    m_update_needed.return_value = True

                    buf = io.StringIO()
                    with contextlib.redirect_stdout(buf):
                        colcon_runner.main(["ia", "--dry-run"])

                    output = buf.getvalue()
                    self.assertIn("rosdep update", output)
                    self.assertNotIn("skipped", output)

                    # Second call - update not needed
                    m_update_needed.return_value = False

                    buf = io.StringIO()
                    with contextlib.redirect_stdout(buf):
                        colcon_runner.main(["ia", "--dry-run"])

                    output = buf.getvalue()
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


if __name__ == "__main__":  # pragma: no cover — run the tests
    unittest.main(verbosity=2)
