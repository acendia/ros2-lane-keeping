"""Tests for the lead-vehicle distance detector."""

import importlib.util
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    'lead_detector',
    REPO_ROOT / 'src/auto_drive_sim/auto_drive_sim/lead_detector.py')
lead_detector = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(lead_detector)

CONFIG = lead_detector.LeadDetectorConfig()
FX = 320.0


def test_distance_from_red_box():
    """A 32 px wide box must be 16 m away."""
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (300, 160), (331, 220), (0, 0, 255), -1)
    distance = lead_detector.estimate_lead_distance(image, FX, CONFIG)
    assert distance == 16.0


def test_no_red_box():
    """A road image without red must give no distance."""
    image = np.full((360, 640, 3), 90, dtype=np.uint8)
    assert lead_detector.estimate_lead_distance(image, FX, CONFIG) is None


def test_small_red_blob_ignored():
    """A blob below the area limit must be ignored."""
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (320, 180), (325, 185), (0, 0, 255), -1)
    assert lead_detector.estimate_lead_distance(image, FX, CONFIG) is None
