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


def test_dedicated_pose_uses_source_time_and_rejects_replays():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    node = evaluator.Evaluator()
    try:
        pose = PoseStamped()
        pose.header.stamp.sec = 1
        pose.pose.position.x = -10.0
        pose.pose.position.y = 15.0
        pose.pose.orientation.w = 1.0
        node._on_model_poses(pose)
        assert node.last_time == 1.0
        assert len(node.samples) == 1
        node._on_model_poses(pose)
        pose.header.stamp.sec = 0
        node._on_model_poses(pose)
        assert len(node.samples) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_readiness_acknowledges_an_actual_scored_pose(tmp_path):
    import json
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.parameter import Parameter
    rclpy.init()
    node = evaluator.Evaluator()
    path = tmp_path / 'ready.json'
    try:
        node.set_parameters([Parameter('ready_file', value=str(path))])
        pose = PoseStamped()
        pose.pose.position.x = -10.0
        pose.pose.position.y = 15.0
        pose.pose.orientation.w = 1.0
        node._on_model_poses(pose)
        assert not path.exists()
        pose.header.stamp.sec = 1
        node._on_model_poses(pose)
        assert json.loads(path.read_text()) == {'ready': True, 'pose_stamp_s': 1.0}
        assert len(node.samples) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stop_diagnostics_retain_observations_and_transitions(tmp_path):
    import json
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init()
    node = evaluator.Evaluator()
    node.output_dir = tmp_path
    try:
        lane = String(data=json.dumps({'image_stamp_s': 1.0, 'valid': False,
                                      'reason': 'sparse_boundaries', 'confidence': 0.0}))
        node._on_lane_diagnostic(lane)
        node._on_lane_diagnostic(lane)
        state = String(data=json.dumps({'stamp_s': 1.1, 'status': 'lane_invalid',
                                       'command_speed_mps': 0.0}))
        node._on_controller_diagnostic(state)
        node._on_controller_diagnostic(state)
        moving = Twist()
        moving.linear.x = 0.5
        node._on_cmd_vel(moving)
        node._on_cmd_vel(Twist())
        assert [event['type'] for event in node.diagnostic_events] == [
            'lane_detection_invalid', 'controller_transition', 'commanded_stop']
        stop = node.diagnostic_events[-1]
        assert stop['lane_observation']['reason'] == 'sparse_boundaries'
        assert stop['controller_observation']['status'] == 'lane_invalid'
        assert 'image' not in stop
    finally:
        node.destroy_node()
        rclpy.shutdown()
