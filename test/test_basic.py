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
        self.assertEqual(colcon_runner._parse_verbs("ba"), (False, [("b", "a")]))
        self.assertEqual(colcon_runner._parse_verbs("boto"), (False, [("b", "o"), ("t", "o")]))

    def test_default_specifier(self):
        # When no specifier is provided, default to "a" (all)
        self.assertEqual(colcon_runner._parse_verbs("b"), (False, [("b", "a")]))
        self.assertEqual(colcon_runner._parse_verbs("t"), (False, [("t", "a")]))
        self.assertEqual(colcon_runner._parse_verbs("c"), (False, [("c", "a")]))

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
        self.assertEqual(colcon_runner._parse_verbs("bt"), (False, [("b", "a"), ("t", "a")]))
        self.assertEqual(colcon_runner._parse_verbs("bto"), (False, [("b", "a"), ("t", "o")]))
        self.assertEqual(colcon_runner._parse_verbs("bobt"), (False, [("b", "o"), ("b", "a"), ("t", "a")]))
        self.assertEqual(colcon_runner._parse_verbs("cbt"), (False, [("c", "a"), ("b", "a"), ("t", "a")]))

    def test_underlay_verb(self):
        # Test underlay verb when 'u' is first
        self.assertEqual(colcon_runner._parse_verbs("u"), (True, []))
        self.assertEqual(colcon_runner._parse_verbs("ub"), (True, [("b", "a")]))
        self.assertEqual(colcon_runner._parse_verbs("uba"), (True, [("b", "a")]))
        self.assertEqual(colcon_runner._parse_verbs("ubo"), (True, [("b", "o")]))
        self.assertEqual(colcon_runner._parse_verbs("ubt"), (True, [("b", "a"), ("t", "a")]))

    def test_upto_specifier(self):
        # Test that 'u' after a verb is still treated as "upto" specifier
        self.assertEqual(colcon_runner._parse_verbs("bu"), (False, [("b", "u")]))
        self.assertEqual(colcon_runner._parse_verbs("tu"), (False, [("t", "u")]))
        self.assertEqual(colcon_runner._parse_verbs("cu"), (False, [("c", "u")]))


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


if __name__ == "__main__":  # pragma: no cover — run the tests
    unittest.main(verbosity=2)
