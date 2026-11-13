"""Tests for the pure teleop helpers."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    'teleop', REPO_ROOT / 'src/auto_drive_sim/auto_drive_sim/teleop.py')
teleop = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(teleop)


def test_parse_segments():
    """Segments must parse into ordered command tuples."""
    segments = teleop.parse_segments('1.5,0,2;0.5,-0.3,1')
    assert segments == [(1.5, 0.0, 2.0), (0.5, -0.3, 1.0)]


def test_parse_segments_rejects_bad_input():
    """Bad segment strings must fail instead of being ignored."""
    with pytest.raises(ValueError):
        teleop.parse_segments('1.5,0')
    with pytest.raises(ValueError):
        teleop.parse_segments('1.5,0,-1')


def test_yaw_from_identity():
    """An identity quaternion must produce zero yaw."""
    quaternion = SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
    assert teleop.yaw_from_quaternion(quaternion) == pytest.approx(0.0)


def test_yaw_from_quarter_turn():
    """A 90 degree rotation must produce pi/2 yaw."""
    angle = math.pi / 2.0
    quaternion = SimpleNamespace(
        w=math.cos(angle / 2.0),
        x=0.0,
        y=0.0,
        z=math.sin(angle / 2.0),
    )
    assert teleop.yaw_from_quaternion(quaternion) == pytest.approx(angle)
