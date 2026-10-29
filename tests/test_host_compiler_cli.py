"""Regression tests for missing compiler paths reported by Windows users."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent.parent


class HostCompilerCliTests(unittest.TestCase):
    def check_missing_compiler(self, explicit):
        with tempfile.TemporaryDirectory() as directory:
            missing = str(Path(directory) / "missing-tcc.exe")
            environment = os.environ.copy()
            command = [sys.executable, str(ROOT / "tests/run_control_tests.py")]
            if explicit:
                command += ["--compiler", missing]
                environment.pop("H7_HOST_CC", None)
            else:
                environment["H7_HOST_CC"] = missing
            result = subprocess.run(command, cwd=ROOT, env=environment,
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn("Compiler not found", result.stderr)
            self.assertIn(missing, result.stderr)
            self.assertNotIn("Traceback", result.stderr)

    def test_explicit_missing_compiler_has_actionable_error(self):
        self.check_missing_compiler(True)

    def test_stale_environment_compiler_has_actionable_error(self):
        self.check_missing_compiler(False)


if __name__ == "__main__":
    unittest.main()
