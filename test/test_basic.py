# ==================================================
# test_cr.py – basic unit tests for the *cr* module
# ==================================================

import contextlib
import io
import tempfile
import unittest
import os
from unittest import mock

# Import the module under a stable name regardless of file location.
# import importlib.util

# spec = importlib.util.spec_from_loader("cr", loader=None)
# cr = importlib.util.module_from_spec(spec)  # type: ignore[assignment]
from colcon_runner import cr

# Inject the code from this file (up to this point) so that the tests have access.
# NOTE: When the file is split into separate modules, simply `import cr` instead.

_module_source = __doc__.split("# ==================================================")[0]
exec(_module_source, cr.__dict__)


class ParseVerbTests(unittest.TestCase):
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
        self.tmp_cfg = tempfile.NamedTemporaryFile(delete=False)
        self.addCleanup(os.unlink, self.tmp_cfg.name)
        os.environ["CR_CONFIG"] = self.tmp_cfg.name

    def test_set_default_and_dry_run(self):
        # Patch subprocess.run so no real commands are executed.
        with mock.patch.object(cr, "subprocess") as m_sp:
            m_sp.run.return_value.returncode = 0

            # Set default package
            cr.main(["s", "demo_pkg"])
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
