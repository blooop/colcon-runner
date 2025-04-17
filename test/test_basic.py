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
from colcon_runner import cr


class ParseVerbTests(unittest.TestCase):
    # pylint: disable=protected-access
    def test_valid_pairs(self):
        self.assertEqual(cr._parse_verbs("ba"), [("b", "a")])
        self.assertEqual(cr._parse_verbs("boto"), [("b", "o"), ("t", "o")])

    def test_invalid_length(self):
        with self.assertRaises(cr.ParseError):
            cr._parse_verbs("b")

    def test_unknown_verb(self):
        with self.assertRaises(cr.ParseError):
            cr._parse_verbs("xa")

    def test_unknown_spec(self):
        with self.assertRaises(cr.ParseError):
            cr._parse_verbs("bz")


class BuildCommandTests(unittest.TestCase):
    # pylint: disable=protected-access
    def test_build_all(self):
        cmd = cr._build_colcon_cmd("b", "a", None)
        self.assertEqual(cmd, ["colcon", "build"])

    def test_test_only(self):
        cmd = cr._build_colcon_cmd("t", "o", "pkg")
        self.assertEqual(cmd, ["colcon", "test", "--packages-select", "pkg"])

    def test_missing_pkg(self):
        with self.assertRaises(cr.ParseError):
            cr._build_colcon_cmd("c", "u", None)


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
        with mock.patch.object(cr, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            # Set default package
            cr.main(["s", "demo_pkg"])

            # Use a public method or mock here if one exists, but for tests it's acceptable to
            # test internal functionality
            # pylint: disable=protected-access
            self.assertEqual(cr._load_default_package(), "demo_pkg")

            # Issue a compound command using the default + dry‑run
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                cr.main(["boto", "--dry-run"])

            output = buf.getvalue()
            self.assertIn("--packages-select demo_pkg", output)
            # subprocess.run should *not* be called when --dry-run is active
            m_sp.run.assert_not_called()


if __name__ == "__main__":  # pragma: no cover — run the tests
    unittest.main(verbosity=2)
