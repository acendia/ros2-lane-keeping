"""Tests for the evaluator geometry helpers."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    'evaluator',
    REPO_ROOT / 'src/auto_drive_sim/auto_drive_sim/evaluator.py')
evaluator = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(evaluator)


def test_yaw_from_identity():
    """Identity must produce zero yaw."""
    quaternion = type('Q', (), {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0})()
    assert evaluator.yaw_from_quaternion(quaternion) == pytest.approx(0.0)


def test_yaw_from_quarter_turn():
    """A 90 degree quaternion must produce pi/2 yaw."""
    angle = math.pi / 2.0
    quaternion = type('Q', (), {
        'w': math.cos(angle / 2.0),
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(angle / 2.0),
    })()
    assert evaluator.yaw_from_quaternion(quaternion) == pytest.approx(angle)


def test_nearest_index():
    """The nearest centerline sample must be selected."""
    x = np.array([0.0, 1.0, 2.0])
    y = np.array([0.0, 0.0, 0.0])
    assert evaluator.nearest_index(1.2, 0.1, x, y) == 1
    assert evaluator.nearest_index(1.8, -0.1, x, y) == 2


def test_signed_lateral_error():
    """Positive lateral error must mean left of a straight centerline."""
    assert evaluator.signed_lateral_error(0.0, 1.0, 0.0, 0.0, 0.0) == 1.0
    assert evaluator.signed_lateral_error(0.0, -1.0, 0.0, 0.0, 0.0) == -1.0


def test_footprint_corners():
    """The four corners must sit at the vehicle extents."""
    corners = evaluator.footprint_corners(0.0, 0.0, 0.0, 3.8, 1.6)
    assert len(corners) == 4
    assert {tuple(round(value, 3) for value in corner)
            for corner in corners} == {
        (-1.9, -0.8), (-1.9, 0.8), (1.9, -0.8), (1.9, 0.8)}


def test_progress_wrap_forward():
    """Crossing the start line must add the wrapped distance."""
    progress = evaluator.advance_progress(133.0, 1.0, 134.0, 133.0)
    assert progress == pytest.approx(135.0)


def test_progress_wrap_backward():
    """Moving backward across the start line must subtract distance."""
    progress = evaluator.advance_progress(1.0, 133.0, 134.0, 1.0)
    assert progress == pytest.approx(-1.0)


def test_progress_straight():
    """Normal forward progress must add the sample delta."""
    progress = evaluator.advance_progress(10.0, 10.5, 134.0, 10.0)
    assert progress == pytest.approx(10.5)
