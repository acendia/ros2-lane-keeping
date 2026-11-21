"""Tests for the lead-vehicle distance detector."""

import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    'lead_detector',
    REPO_ROOT / 'src/auto_drive_sim/auto_drive_sim/lead_detector.py')
lead_detector = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(lead_detector)

_CONFIG = lead_detector.LeadDetectorConfig()
_GROUND = lead_detector.GroundPlane(lead_detector.CameraModel())


def test_distance_from_red_mark():
    """A red ground mark at 16 m must give about 16 m."""
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    points = []
    for y_m in (-0.7, 0.7):
        pixel = _GROUND.project(16.0, y_m)
        if pixel is not None:
            points.append(pixel)
    cv2.line(image, points[0], points[1], (0, 0, 255), 2)
    distance = lead_detector.estimate_lead_distance(
        image, _GROUND, _CONFIG)
    assert distance == pytest.approx(16.0, abs=1.0)


def test_no_red_mark():
    """A road image without red must give no distance."""
    image = np.full((360, 640, 3), 90, dtype=np.uint8)
    assert lead_detector.estimate_lead_distance(
        image, _GROUND, _CONFIG) is None


def test_small_red_blob_ignored():
    """A blob below the red-pixel limit must be ignored."""
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (320, 180), (322, 182), (0, 0, 255), -1)
    assert lead_detector.estimate_lead_distance(
        image, _GROUND, _CONFIG) is None


def test_node_preserves_image_time_and_publishes_clear_and_invalid():
    import rclpy
    from cv_bridge import CvBridge
    rclpy.init()
    node = lead_detector.LeadDetector()
    messages = []
    class Capture:
        def publish(self, message):
            messages.append(message)
    node.publisher = Capture()
    try:
        image = CvBridge().cv2_to_imgmsg(np.zeros((360, 640, 3), dtype=np.uint8), 'bgr8')
        image.header.stamp = node.get_clock().now().to_msg()
        node._on_image(image)
        assert messages[-1].valid
        assert not messages[-1].detected
        assert messages[-1].header.stamp == image.header.stamp
        image.header.stamp.sec -= 2
        node._on_image(image)
        assert not messages[-1].valid
        assert messages[-1].header.stamp == image.header.stamp
    finally:
        node.destroy_node()
        rclpy.shutdown()
