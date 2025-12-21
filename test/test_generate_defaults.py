# ==================================================
# test_generate_defaults.py – tests for generate_defaults module
# ==================================================

import unittest
import tempfile
import os
import yaml
from colcon_runner import generate_defaults


class GenerateDefaultsTests(unittest.TestCase):
    def test_generate_basic_structure(self):
        """Test that generated YAML has the expected structure."""
        content = generate_defaults.generate_colcon_defaults("/test/workspace")
        data = yaml.safe_load(content)

        # Check that main sections exist
        self.assertIn("build", data)
        self.assertIn("test", data)
        self.assertIn("test-result", data)
        self.assertIn("clean.workspace", data)
        self.assertIn("clean.packages", data)
        self.assertIn("", data)

    def test_paths_use_workspace_parameter(self):
        """Test that all paths use the provided workspace parameter."""
        workspace = "/my/custom/workspace"
        content = generate_defaults.generate_colcon_defaults(workspace)
        data = yaml.safe_load(content)

        # Verify paths contain the workspace path
        self.assertEqual(data["build"]["build-base"], f"{workspace}/build")
        self.assertEqual(data["build"]["install-base"], f"{workspace}/install")
        self.assertIn(f"{workspace}/src", data["build"]["base-paths"])

    def test_yaml_anchors_resolve_correctly(self):
        """Test that YAML anchors resolve to the same values."""
        workspace = "/test/ws"
        content = generate_defaults.generate_colcon_defaults(workspace)
        data = yaml.safe_load(content)

        # Verify that anchors resolve correctly
        build_base = data["build"]["build-base"]
        test_build_base = data["test"]["build-base"]
        clean_build_base = data["clean.workspace"]["build-base"]

        self.assertEqual(build_base, test_build_base)
        self.assertEqual(build_base, clean_build_base)
        self.assertEqual(build_base, f"{workspace}/build")

    def test_clean_workspace_and_packages_are_identical(self):
        """Test that clean.workspace and clean.packages configs are identical."""
        content = generate_defaults.generate_colcon_defaults("/test/ws")
        data = yaml.safe_load(content)

        # These should be identical due to the anchor
        self.assertEqual(data["clean.workspace"], data["clean.packages"])

    def test_path_normalization(self):
        """Test that paths are normalized (no trailing slashes)."""
        # Test with trailing slash
        content = generate_defaults.generate_colcon_defaults("/test/workspace/")
        self.assertIn("/test/workspace/build", content)
        self.assertNotIn("/test/workspace//build", content)

    def test_main_with_current_directory(self):
        """Test main function with current directory."""
        import sys
        from io import StringIO

        captured_output = StringIO()
        sys.stdout = captured_output

        try:
            generate_defaults.main(["."])
            output = captured_output.getvalue()

            # Verify output is valid YAML
            data = yaml.safe_load(output)
            self.assertIn("build", data)
            self.assertIn("test", data)
        finally:
            sys.stdout = sys.__stdout__

    def test_main_with_nonexistent_path(self):
        """Test main function with nonexistent path exits with error."""
        with self.assertRaises(SystemExit) as cm:
            generate_defaults.main(["/this/path/does/not/exist"])
        self.assertEqual(cm.exception.code, 1)

    def test_main_with_output_file(self):
        """Test main function with output file option."""
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace = tmpdir
            output_file = os.path.join(tmpdir, "test_defaults.yaml")

            generate_defaults.main([workspace, "-o", output_file])

            # Verify file was created
            self.assertTrue(os.path.exists(output_file))

            # Verify file content is valid YAML
            with open(output_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
                self.assertIn("build", data)
                self.assertEqual(data["build"]["build-base"], f"{workspace}/build")

    def test_cmake_args_present(self):
        """Test that cmake-args are included in build config."""
        content = generate_defaults.generate_colcon_defaults("/test/ws")
        data = yaml.safe_load(content)

        self.assertIn("cmake-args", data["build"])
        cmake_args = data["build"]["cmake-args"]
        self.assertIn("-DCMAKE_BUILD_TYPE=RelWithDebInfo", cmake_args)
        self.assertIn("-DCMAKE_EXPORT_COMPILE_COMMANDS=ON", cmake_args)

    def test_symlink_install_enabled(self):
        """Test that symlink-install is set to true."""
        content = generate_defaults.generate_colcon_defaults("/test/ws")
        data = yaml.safe_load(content)

        self.assertTrue(data["build"]["symlink-install"])


if __name__ == "__main__":
    unittest.main()
