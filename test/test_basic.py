# ==================================================
# test_cr.py – basic unit tests for the *cr* module
# ==================================================

import contextlib
import io
import tempfile
import unittest
import os
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
            cmd, ["install", "--from-paths", "/fake/workspace/src", "--ignore-src", "-y", "-r"]
        )

    def test_install_only(self):
        cmd = colcon_runner._build_rosdep_cmd("o", "pkg")
        self.assertEqual(
            cmd, ["install", "--from-paths", "/fake/workspace/src/pkg", "--ignore-src", "-y", "-r"]
        )

    def test_install_upto(self):
        cmd = colcon_runner._build_rosdep_cmd("u", "pkg")
        self.assertEqual(
            cmd, ["install", "--from-paths", "/fake/workspace/src/pkg", "--ignore-src", "-y", "-r"]
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
            self.assertIn("rosdep install --from-paths /test/workspace/src --ignore-src -y -r", output)

            # Test install only with package
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                colcon_runner.main(["io", "test_pkg", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("rosdep update", output)
            self.assertIn(
                "rosdep install --from-paths /test/workspace/src/test_pkg --ignore-src -y -r",
                output,
            )

            # subprocess.run should *not* be called when --dry-run is active
            m_sp.run.assert_not_called()


if __name__ == "__main__":  # pragma: no cover — run the tests
    unittest.main(verbosity=2)
