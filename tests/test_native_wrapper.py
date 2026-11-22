"""The shell wrapper must fail clearly before executing an invalid environment."""
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_missing_framework_python_has_actionable_error(tmp_path):
    env = dict(os.environ, NATIVE_FRAMEWORK_PYTHON=str(tmp_path / 'missing'))
    result = subprocess.run(['bash', str(ROOT / 'tools/native.sh'), 'true'],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert 'Missing framework Python' in result.stderr


def test_missing_bundle_has_actionable_error(tmp_path):
    python = tmp_path / 'python'
    python.write_text('#!/bin/sh\nexit 0\n')
    python.chmod(0o755)
    env = dict(os.environ, NATIVE_FRAMEWORK_PYTHON=str(python), NATIVE_ROS_PREFIX=str(tmp_path / 'missing'))
    result = subprocess.run(['bash', str(ROOT / 'tools/native.sh'), 'true'],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert 'Missing native ROS bundle' in result.stderr
