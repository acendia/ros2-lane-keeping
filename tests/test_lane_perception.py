"""Tests for the pure lane perception pipeline."""

import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
FRAMES = REPO_ROOT / 'tests/frames'
_SPEC = importlib.util.spec_from_file_location(
    'lane_perception',
    REPO_ROOT / 'src/auto_drive_sim/auto_drive_sim/lane_perception.py')
lane_perception = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(lane_perception)

CAMERA = lane_perception.CameraModel()
CONFIG = lane_perception.PerceptionConfig()
GROUND = lane_perception.GroundPlane(CAMERA)


def _frame(name: str) -> np.ndarray:
    image = cv2.imread(str(FRAMES / name))
    assert image is not None, f'missing frame {name}'
    return image


def _fit(value: float) -> lane_perception.BoundaryFit:
    return lane_perception.BoundaryFit(
        coefficients=np.array([0.0, 0.0, value]),
        rms_m=0.01,
        point_count=100,
        x_min_m=4.0,
        x_max_m=12.0,
    )


def test_straight_frame():
    """The straight frame must give a centered lane."""
    result = lane_perception.analyze_frame(
        _frame('straight.png'), GROUND, CONFIG)
    assert result.valid
    assert result.reason == ''
    assert result.lateral_error_m == pytest.approx(0.0, abs=0.1)
    assert result.lane_width_m == pytest.approx(3.66, abs=0.15)
    assert result.target_x_m == pytest.approx(5.4)
    assert result.confidence > 0.7


def test_right_curve_frame():
    """The right curve must be valid and offset to the left of the camera."""
    result = lane_perception.analyze_frame(
        _frame('right_curve.png'), GROUND, CONFIG)
    assert result.valid
    assert result.lateral_error_m == pytest.approx(1.07, abs=0.3)
    assert result.lane_width_m == pytest.approx(3.69, abs=0.2)
    assert result.target_x_m == pytest.approx(5.4)


def test_left_curve_frame():
    """The left curve must be valid and offset to the right of the camera."""
    result = lane_perception.analyze_frame(
        _frame('left_curve.png'), GROUND, CONFIG)
    assert result.valid
    assert result.lateral_error_m == pytest.approx(-1.06, abs=0.3)
    assert result.lane_width_m == pytest.approx(3.69, abs=0.2)
    assert result.target_x_m == pytest.approx(5.4)


def test_no_line_frame():
    """A grass-only frame must not invent a lane."""
    result = lane_perception.analyze_frame(
        _frame('no_line.png'), GROUND, CONFIG)
    assert not result.valid
    assert result.reason == 'no_lane_markings'


def test_blank_frame():
    """A uniform frame must not invent a lane."""
    image = np.full((360, 640, 3), 128, dtype=np.uint8)
    result = lane_perception.analyze_frame(image, GROUND, CONFIG)
    assert not result.valid
    assert result.reason == 'no_lane_markings'


def test_one_boundary_frame():
    """Removing one boundary must invalidate the estimate."""
    image = _frame('straight.png')
    image[:, :330] = (60, 60, 60)
    result = lane_perception.analyze_frame(image, GROUND, CONFIG)
    assert not result.valid
    assert result.reason in ('one_boundary', 'no_lane_markings')


def test_narrow_lane_rejected():
    """An implausibly narrow lane must be rejected."""
    assert lane_perception.validate_boundaries(
        _fit(1.0), _fit(-1.0), CONFIG) == 'lane_too_narrow'


def test_wide_lane_rejected():
    """An implausibly wide lane must be rejected."""
    assert lane_perception.validate_boundaries(
        _fit(4.0), _fit(-4.0), CONFIG) == 'lane_too_wide'


def test_swapped_boundaries_rejected():
    """Reversed left and right fits must be rejected."""
    assert lane_perception.validate_boundaries(
        _fit(-1.8), _fit(1.8), CONFIG) == 'swapped_boundaries'


def test_good_boundaries_accepted():
    """A plausible pair must pass validation."""
    assert lane_perception.validate_boundaries(
        _fit(1.81), _fit(-1.81), CONFIG) == ''


def test_sparse_fit_rejected():
    """Too few boundary points must not produce a fit."""
    x_values = np.linspace(4.0, 5.0, 10)
    y_values = np.zeros_like(x_values)
    assert lane_perception.fit_boundary(
        x_values, y_values, CONFIG) is None


def test_stale_image():
    """Images older than the limit must be stale."""
    assert lane_perception.is_stale(10.0, 10.4, 0.3)
    assert not lane_perception.is_stale(10.0, 10.2, 0.3)
    assert lane_perception.is_stale(10.0, 9.0, 0.3)


def test_debug_image_shape():
    """The overlay must keep the source image shape."""
    image = _frame('straight.png')
    result = lane_perception.analyze_frame(image, GROUND, CONFIG)
    debug = lane_perception.make_debug_image(
        image, result, GROUND, CONFIG)
    assert debug.shape == image.shape
    assert debug.dtype == np.uint8
