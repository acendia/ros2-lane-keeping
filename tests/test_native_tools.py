"""Check native validation process cleanup and failure propagation."""
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('native_check', ROOT / 'tools/native_check.py')
native_check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(native_check)


def test_process_cleanup_terminates_only_owned_child(tmp_path):
    processes = native_check.Processes(tmp_path)
    child = processes.start('sleep', [sys.executable, '-c', 'import time; time.sleep(60)'])
    processes.close()
    assert child.poll() is not None


def test_command_failure_is_not_treated_as_validation_success():
    with pytest.raises(subprocess.CalledProcessError) as error:
        native_check.run([sys.executable, '-c', 'raise SystemExit(7)'])
    assert error.value.returncode == 7
