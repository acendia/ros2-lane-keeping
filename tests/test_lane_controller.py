"""Tests for the pure-pursuit controller helpers."""

import importlib.util
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    'lane_controller',
    REPO_ROOT / 'src/auto_drive_sim/auto_drive_sim/lane_controller.py')
lane_controller = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(lane_controller)

CONFIG = lane_controller.ControllerConfig(
    target_speed_mps=1.5,
    max_acceleration_mps2=1.0,
    max_curvature_1pm=0.2,
    max_yaw_rate_radps=0.6,
    lookahead_gain=1.0,
    yaw_smoothing_alpha=1.0,
)


def _update(follower, now, valid=True, target=(5.4, 0.0),
            estimate_stamp=None, odom_stamp=None,
            lead_distance=None, lead_age=0.0):
    if estimate_stamp is None:
        estimate_stamp = now
    if odom_stamp is None:
        odom_stamp = now
    return follower.update(
        now, True, valid, estimate_stamp,
        target[0], target[1], True, odom_stamp,
        lead_distance, lead_age)


def _ramp(follower, seconds=2.0, target=(5.4, 0.0)):
    command = None
    for step in range(int(seconds / 0.1) + 1):
        command = _update(follower, step * 0.1, target=target)
    return command


FAST_CONFIG = lane_controller.ControllerConfig(
    target_speed_mps=2.0,
    max_acceleration_mps2=1.0,
    max_deceleration_mps2=1.0,
    max_lateral_acceleration_mps2=0.15,
    min_curve_speed_mps=0.6,
    max_curvature_1pm=0.2,
    lookahead_gain=1.0,
    yaw_smoothing_alpha=1.0,
)


def test_speed_limit_straight():
    """A straight must allow the full target speed."""
    assert lane_controller.speed_limit_for_curvature(
        0.0, FAST_CONFIG) == 2.0


def test_speed_limit_curve():
    """The 15 m radius must cap speed at 1.5 m/s."""
    assert lane_controller.speed_limit_for_curvature(
        1.0 / 15.0, FAST_CONFIG) == pytest.approx(1.5)


def test_speed_limit_floor():
    """A very sharp curve must use the minimum speed floor."""
    assert lane_controller.speed_limit_for_curvature(
        10.0, FAST_CONFIG) == pytest.approx(0.6)


def test_curve_deceleration():
    """Speed must decelerate at the configured rate into a curve."""
    follower = lane_controller.LaneFollower(FAST_CONFIG)
    _ramp(follower, seconds=2.0, target=(5.4, 0.0))
    first = _update(follower, 2.1, target=(5.4, 1.0))
    for step in range(2, 8):
        later = _update(follower, 2.1 + (step - 1) * 0.1,
                        target=(5.4, 1.0))
    assert first.speed_mps == pytest.approx(1.9)
    assert later.speed_mps == pytest.approx(1.5, abs=0.01)


def test_curvature_straight():
    """A centered lookahead point must give zero curvature."""
    assert lane_controller.curvature_for_point(5.4, 0.0, 0.2) == 0.0


def test_curvature_left_and_right():
    """Left and right targets must have opposite curvature signs."""
    left = lane_controller.curvature_for_point(5.4, 1.0, 0.2)
    right = lane_controller.curvature_for_point(5.4, -1.0, 0.2)
    assert left == pytest.approx(0.0664, abs=0.001)
    assert right == pytest.approx(-0.0664, abs=0.001)


def test_curvature_is_bounded():
    """Excessive lateral targets must hit the curvature limit."""
    assert lane_controller.curvature_for_point(5.4, 5.4, 0.15) == 0.15
    assert lane_controller.curvature_for_point(5.4, -5.4, 0.15) == -0.15


def test_bad_lookahead():
    """A target behind the car must be rejected."""
    follower = lane_controller.LaneFollower(CONFIG)
    command = _update(follower, 0.0, target=(0.0, 0.0))
    assert command.status == 'bad_lookahead'
    assert command.speed_mps == 0.0


def test_speed_ramp():
    """Speed must ramp instead of jumping to target."""
    follower = lane_controller.LaneFollower(CONFIG)
    first = _update(follower, 0.0)
    second = _update(follower, 0.1)
    after = _ramp(follower)
    assert first.speed_mps == 0.0
    assert second.speed_mps == pytest.approx(0.1)
    assert after.speed_mps == pytest.approx(1.5)


def test_straight_command():
    """A centered target must drive straight with zero yaw."""
    follower = lane_controller.LaneFollower(CONFIG)
    command = _ramp(follower)
    assert command.status == 'driving'
    assert command.speed_mps == pytest.approx(1.5)
    assert command.yaw_rate_radps == pytest.approx(0.0, abs=1e-6)


def test_left_command():
    """A left target must produce positive yaw rate."""
    follower = lane_controller.LaneFollower(CONFIG)
    command = _ramp(follower, target=(5.4, 1.0))
    assert command.yaw_rate_radps == pytest.approx(
        1.5 * 0.0664, abs=0.005)
    assert command.yaw_rate_radps > 0.0


def test_right_command():
    """A right target must produce negative yaw rate."""
    follower = lane_controller.LaneFollower(CONFIG)
    command = _ramp(follower, target=(5.4, -1.0))
    assert command.yaw_rate_radps == pytest.approx(
        -1.5 * 0.0664, abs=0.005)
    assert command.yaw_rate_radps < 0.0


def test_lead_speed_cap():
    """A nearby lead vehicle must cap the speed."""
    config = lane_controller.ControllerConfig(
        target_speed_mps=2.0,
        max_acceleration_mps2=10.0,
        max_deceleration_mps2=10.0,
        max_lateral_acceleration_mps2=100.0,
        min_curve_speed_mps=0.6,
        max_curvature_1pm=0.2,
        lookahead_gain=1.0,
        yaw_smoothing_alpha=1.0,
    )
    follower = lane_controller.LaneFollower(config)
    _ramp(follower, seconds=1.0)
    command = _update(follower, 2.0, lead_distance=8.0)
    assert command.speed_mps == pytest.approx(1.6)


def test_lead_stop():
    """A lead vehicle inside the gap must stop the car."""
    follower = lane_controller.LaneFollower(lane_controller.ControllerConfig())
    _ramp(follower)
    command = _update(follower, 2.0, lead_distance=5.5)
    assert command.status == 'lead_vehicle'
    assert command.speed_mps == 0.0


def test_yaw_acceleration_limit():
    """A large steering jump must be rate limited."""
    config = lane_controller.ControllerConfig(
        target_speed_mps=2.0,
        max_acceleration_mps2=100.0,
        max_deceleration_mps2=100.0,
        max_lateral_acceleration_mps2=100.0,
        min_curve_speed_mps=0.6,
        max_curvature_1pm=0.2,
        max_yaw_rate_radps=0.6,
        max_yaw_acceleration_radps2=1.0,
        lookahead_gain=1.0,
        yaw_smoothing_alpha=1.0,
    )
    follower = lane_controller.LaneFollower(config)
    _update(follower, 0.0, target=(5.4, 5.4))
    first = _update(follower, 0.1, target=(5.4, 5.4))
    second = _update(follower, 0.2, target=(5.4, 5.4))
    assert first.yaw_rate_radps == pytest.approx(0.1)
    assert second.yaw_rate_radps == pytest.approx(0.2)


def test_long_loss_recovery_cap():
    """A long stop must recover under the reduced speed cap."""
    config = lane_controller.ControllerConfig(
        target_speed_mps=2.0,
        max_acceleration_mps2=10.0,
        max_deceleration_mps2=10.0,
        max_lateral_acceleration_mps2=100.0,
        min_curve_speed_mps=0.6,
        max_curvature_1pm=0.2,
        max_yaw_rate_radps=0.6,
        lookahead_gain=1.0,
        yaw_smoothing_alpha=1.0,
        long_loss_s=0.3,
        recovery_speed_mps=0.8,
        recovery_hold_s=0.5,
    )
    follower = lane_controller.LaneFollower(config)
    _ramp(follower, seconds=1.0)
    _update(follower, 2.0, valid=False)
    recovered = _update(follower, 2.4)
    held = _update(follower, 2.5)
    released = _update(follower, 3.0)
    assert recovered.speed_mps == pytest.approx(0.8)
    assert held.speed_mps == pytest.approx(0.8)
    assert released.speed_mps == pytest.approx(2.0)


def test_yaw_rate_limit():
    """The yaw-rate command must respect its own limit."""
    config = lane_controller.ControllerConfig(
        target_speed_mps=1.5,
        max_acceleration_mps2=1.0,
        max_curvature_1pm=0.2,
        max_yaw_rate_radps=0.1,
        lookahead_gain=1.0,
        yaw_smoothing_alpha=1.0,
    )
    follower = lane_controller.LaneFollower(config)
    command = _ramp(follower, target=(5.4, 20.0))
    assert command.yaw_rate_radps == pytest.approx(0.1)


def test_stale_lane_estimate():
    """An old lane estimate must stop the car."""
    follower = lane_controller.LaneFollower(CONFIG)
    _ramp(follower)
    command = _update(follower, 2.0, estimate_stamp=0.0)
    assert command.status == 'stale_lane_estimate'
    assert command.speed_mps == 0.0
    assert command.yaw_rate_radps == 0.0


def test_stale_odometry():
    """Old odometry must stop the car."""
    follower = lane_controller.LaneFollower(CONFIG)
    _ramp(follower)
    command = _update(follower, 2.0, odom_stamp=0.0)
    assert command.status == 'stale_odometry'
    assert command.speed_mps == 0.0


def test_lost_lane():
    """An invalid lane estimate must stop the car."""
    follower = lane_controller.LaneFollower(CONFIG)
    _ramp(follower)
    command = _update(follower, 2.0, valid=False)
    assert command.status == 'lane_invalid'
    assert command.speed_mps == 0.0


def test_recovery_ramp():
    """Speed must ramp from zero after a stop."""
    follower = lane_controller.LaneFollower(CONFIG)
    _ramp(follower)
    stopped = _update(follower, 2.0, valid=False)
    recovered = _update(follower, 2.1)
    later = _update(follower, 2.2)
    assert stopped.speed_mps == 0.0
    assert recovered.speed_mps == pytest.approx(0.1)
    assert later.speed_mps == pytest.approx(0.2)
    assert recovered.speed_mps < CONFIG.target_speed_mps


def test_yaw_smoothing():
    """Yaw must not jump to the target on the first movement tick."""
    config = lane_controller.ControllerConfig(
        target_speed_mps=1.5,
        max_acceleration_mps2=100.0,
        max_curvature_1pm=0.2,
        max_yaw_rate_radps=0.6,
        lookahead_gain=1.0,
        yaw_smoothing_alpha=0.3,
    )
    follower = lane_controller.LaneFollower(config)
    _update(follower, 0.0, target=(5.4, 1.0))
    first = _update(follower, 0.1, target=(5.4, 1.0))
    target_yaw = 1.5 * 0.0664
    assert first.yaw_rate_radps == pytest.approx(0.3 * target_yaw, abs=0.005)
    for step in range(2, 60):
        first = _update(follower, step * 0.1, target=(5.4, 1.0))
    assert first.yaw_rate_radps == pytest.approx(target_yaw, abs=0.005)
