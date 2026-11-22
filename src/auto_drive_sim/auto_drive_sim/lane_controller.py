"""Pure-pursuit lane following controller."""

from dataclasses import dataclass
import json
import math
from typing import Optional

from auto_drive_sim_interfaces.msg import LaneEstimate
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from auto_drive_sim_interfaces.msg import LeadEstimate


@dataclass(frozen=True)
class ControllerConfig:
    """Controller limits and smoothing."""

    target_speed_mps: float = 1.5
    max_acceleration_mps2: float = 0.6
    max_deceleration_mps2: float = 1.0
    max_lateral_acceleration_mps2: float = 0.15
    min_curve_speed_mps: float = 0.6
    max_curvature_1pm: float = 0.15
    max_yaw_rate_radps: float = 0.6
    max_yaw_acceleration_radps2: float = 1.5
    lookahead_gain: float = 1.2
    yaw_smoothing_alpha: float = 0.35
    long_loss_s: float = 0.5
    recovery_speed_mps: float = 0.8
    recovery_hold_s: float = 1.0
    lead_enabled: bool = False
    lead_follow_gap_m: float = 6.0
    lead_speed_gain: float = 0.8
    lead_timeout_s: float = 0.5
    max_lane_age_s: float = 0.3
    max_odom_age_s: float = 0.3
    command_rate_hz: float = 20.0

    def __post_init__(self) -> None:
        """Validate the parameter ranges."""
        if self.target_speed_mps <= 0.0:
            raise ValueError('target_speed_mps must be positive')
        if self.max_acceleration_mps2 <= 0.0:
            raise ValueError('max_acceleration_mps2 must be positive')
        if self.max_deceleration_mps2 <= 0.0:
            raise ValueError('max_deceleration_mps2 must be positive')
        if self.max_lateral_acceleration_mps2 <= 0.0:
            raise ValueError('max_lateral_acceleration_mps2 must be positive')
        if not 0.0 < self.min_curve_speed_mps <= self.target_speed_mps:
            raise ValueError(
                'min_curve_speed_mps must be in (0, target_speed_mps]')
        if self.max_curvature_1pm <= 0.0:
            raise ValueError('max_curvature_1pm must be positive')
        if self.max_yaw_rate_radps <= 0.0:
            raise ValueError('max_yaw_rate_radps must be positive')
        if self.max_yaw_acceleration_radps2 <= 0.0:
            raise ValueError('max_yaw_acceleration_radps2 must be positive')
        if self.long_loss_s < 0.0:
            raise ValueError('long_loss_s must not be negative')
        if not 0.0 < self.recovery_speed_mps <= self.target_speed_mps:
            raise ValueError(
                'recovery_speed_mps must be in (0, target_speed_mps]')
        if self.recovery_hold_s < 0.0:
            raise ValueError('recovery_hold_s must not be negative')
        if self.lead_follow_gap_m <= 0.0:
            raise ValueError('lead_follow_gap_m must be positive')
        if self.lead_speed_gain <= 0.0:
            raise ValueError('lead_speed_gain must be positive')
        if self.lead_timeout_s < 0.0:
            raise ValueError('lead_timeout_s must not be negative')
        if self.lookahead_gain <= 0.0:
            raise ValueError('lookahead_gain must be positive')
        if self.command_rate_hz <= 0.0:
            raise ValueError('command_rate_hz must be positive')
        if not 0.05 <= self.yaw_smoothing_alpha <= 1.0:
            raise ValueError('yaw_smoothing_alpha must be in [0.05, 1.0]')


@dataclass(frozen=True)
class ControllerCommand:
    """One speed and yaw-rate command."""

    speed_mps: float
    yaw_rate_radps: float
    status: str


def curvature_for_point(x_m: float, y_m: float,
                        max_curvature_1pm: float) -> Optional[float]:
    """Return bounded pure-pursuit curvature for a lookahead point."""
    if not math.isfinite(x_m) or not math.isfinite(y_m) or x_m <= 0.1:
        return None
    distance_sq = x_m * x_m + y_m * y_m
    curvature = 2.0 * y_m / distance_sq
    return max(-max_curvature_1pm,
               min(max_curvature_1pm, curvature))


def speed_limit_for_curvature(curvature: float,
                              config: ControllerConfig) -> float:
    """Return the speed cap for a curvature from lateral acceleration."""
    if abs(curvature) < 1e-9:
        return config.target_speed_mps
    limit = math.sqrt(
        config.max_lateral_acceleration_mps2 / abs(curvature))
    return min(
        config.target_speed_mps,
        max(config.min_curve_speed_mps, limit))


class LaneFollower:
    """Stateful speed ramp, pure-pursuit curvature, and yaw smoothing."""

    def __init__(self, config: ControllerConfig) -> None:
        """Create a follower with zero speed and yaw."""
        self.config = config
        self.speed_mps = 0.0
        self.yaw_rate_radps = 0.0
        self.last_update_s = None
        self.stop_started_s = None
        self.recovery_until_s = None

    def update(self, now_s: float, estimate_present: bool,
               estimate_valid: bool, estimate_stamp_s: float,
               target_x_m: float, target_y_m: float,
               odom_present: bool, odom_stamp_s: float,
               lead_distance_m: Optional[float] = None,
               lead_age_s: Optional[float] = None) -> ControllerCommand:
        """Update the controller and return the current command."""
        dt = 0.0
        if self.last_update_s is not None:
            dt = max(0.0, min(0.2, now_s - self.last_update_s))
        self.last_update_s = now_s

        reason = self._stop_reason(
            now_s, estimate_present, estimate_valid, estimate_stamp_s,
            odom_present, odom_stamp_s)
        if not reason and self.config.lead_enabled and (
                lead_age_s is None or not 0.0 <= lead_age_s <= self.config.lead_timeout_s):
            reason = 'stale_lead_estimate'
        if reason:
            if self.stop_started_s is None:
                self.stop_started_s = now_s
            self.speed_mps = 0.0
            self.yaw_rate_radps = 0.0
            return ControllerCommand(0.0, 0.0, reason)

        if self.stop_started_s is not None:
            stop_duration = now_s - self.stop_started_s
            if stop_duration >= self.config.long_loss_s:
                self.recovery_until_s = now_s + self.config.recovery_hold_s
            self.stop_started_s = None

        lookahead_x = target_x_m * self.config.lookahead_gain
        lookahead_y = target_y_m * self.config.lookahead_gain
        curvature = curvature_for_point(
            lookahead_x, lookahead_y, self.config.max_curvature_1pm)
        if curvature is None:
            self.speed_mps = 0.0
            self.yaw_rate_radps = 0.0
            return ControllerCommand(0.0, 0.0, 'bad_lookahead')

        speed_limit = speed_limit_for_curvature(curvature, self.config)
        if self.recovery_until_s is not None:
            if now_s < self.recovery_until_s:
                speed_limit = min(
                    speed_limit, self.config.recovery_speed_mps)
            else:
                self.recovery_until_s = None
        if lead_distance_m is not None and lead_age_s is not None and \
                lead_age_s <= self.config.lead_timeout_s:
            lead_limit = max(
                0.0,
                (lead_distance_m - self.config.lead_follow_gap_m) *
                self.config.lead_speed_gain)
            speed_limit = min(speed_limit, lead_limit)
            if lead_limit <= 0.01:
                self.speed_mps = 0.0
                self.yaw_rate_radps = 0.0
                return ControllerCommand(0.0, 0.0, 'lead_vehicle')
        if self.speed_mps < speed_limit:
            self.speed_mps = min(
                speed_limit,
                self.speed_mps +
                self.config.max_acceleration_mps2 * dt)
        else:
            self.speed_mps = max(
                speed_limit,
                self.speed_mps -
                self.config.max_deceleration_mps2 * dt)
        target_yaw = self.speed_mps * curvature
        target_yaw = max(-self.config.max_yaw_rate_radps,
                         min(self.config.max_yaw_rate_radps, target_yaw))
        alpha = self.config.yaw_smoothing_alpha
        smoothed_yaw = (
            alpha * target_yaw + (1.0 - alpha) * self.yaw_rate_radps)
        max_yaw_step = self.config.max_yaw_acceleration_radps2 * dt
        self.yaw_rate_radps = max(
            self.yaw_rate_radps - max_yaw_step,
            min(self.yaw_rate_radps + max_yaw_step, smoothed_yaw))
        return ControllerCommand(
            round(self.speed_mps, 4),
            round(self.yaw_rate_radps, 4),
            'driving')

    def _stop_reason(self, now_s: float, estimate_present: bool,
                     estimate_valid: bool, estimate_stamp_s: float,
                     odom_present: bool, odom_stamp_s: float) -> str:
        if not estimate_present:
            return 'no_lane_estimate'
        if not estimate_valid:
            return 'lane_invalid'
        if now_s - estimate_stamp_s > self.config.max_lane_age_s:
            return 'stale_lane_estimate'
        if not odom_present:
            return 'no_odometry'
        if now_s - odom_stamp_s > self.config.max_odom_age_s:
            return 'stale_odometry'
        return ''


class LaneController(Node):
    """ROS node that turns LaneEstimate into bounded Twist commands."""

    def __init__(self) -> None:
        """Create subscriptions, the command timer, and the publisher."""
        super().__init__('lane_controller')
        self.declare_parameter('target_speed_mps', 1.5)
        self.declare_parameter('max_acceleration_mps2', 0.6)
        self.declare_parameter('max_deceleration_mps2', 1.0)
        self.declare_parameter('max_lateral_acceleration_mps2', 0.15)
        self.declare_parameter('min_curve_speed_mps', 0.6)
        self.declare_parameter('max_curvature_1pm', 0.15)
        self.declare_parameter('max_yaw_rate_radps', 0.6)
        self.declare_parameter('max_yaw_acceleration_radps2', 1.5)
        self.declare_parameter('lookahead_gain', 1.2)
        self.declare_parameter('yaw_smoothing_alpha', 0.35)
        self.declare_parameter('long_loss_s', 0.5)
        self.declare_parameter('recovery_speed_mps', 0.8)
        self.declare_parameter('recovery_hold_s', 1.0)
        self.declare_parameter('lead_enabled', False)
        self.declare_parameter('lead_follow_gap_m', 6.0)
        self.declare_parameter('lead_speed_gain', 0.8)
        self.declare_parameter('lead_timeout_s', 0.5)
        self.declare_parameter('max_lane_age_s', 0.3)
        self.declare_parameter('max_odom_age_s', 0.3)
        self.declare_parameter('command_rate_hz', 20.0)

        self.config = ControllerConfig(
            lead_enabled=self.get_parameter('lead_enabled').value,
            lead_follow_gap_m=self.get_parameter('lead_follow_gap_m').value,
            lead_speed_gain=self.get_parameter('lead_speed_gain').value,
            lead_timeout_s=self.get_parameter('lead_timeout_s').value,
            target_speed_mps=self.get_parameter(
                'target_speed_mps').value,
            max_acceleration_mps2=self.get_parameter(
                'max_acceleration_mps2').value,
            max_deceleration_mps2=self.get_parameter(
                'max_deceleration_mps2').value,
            max_lateral_acceleration_mps2=self.get_parameter(
                'max_lateral_acceleration_mps2').value,
            min_curve_speed_mps=self.get_parameter(
                'min_curve_speed_mps').value,
            max_curvature_1pm=self.get_parameter(
                'max_curvature_1pm').value,
            max_yaw_rate_radps=self.get_parameter(
                'max_yaw_rate_radps').value,
            max_yaw_acceleration_radps2=self.get_parameter(
                'max_yaw_acceleration_radps2').value,
            lookahead_gain=self.get_parameter(
                'lookahead_gain').value,
            yaw_smoothing_alpha=self.get_parameter(
                'yaw_smoothing_alpha').value,
            long_loss_s=self.get_parameter(
                'long_loss_s').value,
            recovery_speed_mps=self.get_parameter(
                'recovery_speed_mps').value,
            recovery_hold_s=self.get_parameter(
                'recovery_hold_s').value,
            max_lane_age_s=self.get_parameter(
                'max_lane_age_s').value,
            max_odom_age_s=self.get_parameter(
                'max_odom_age_s').value,
            command_rate_hz=self.get_parameter(
                'command_rate_hz').value,
        )
        self.follower = LaneFollower(self.config)
        self.estimate = None
        self.odometry = None
        self.lead_distance = None
        self.lead_stamp_s = None
        self.last_status = ''
        self.publisher = self.create_publisher(Twist, '/car/cmd_vel', 10)
        self.diagnostic_publisher = self.create_publisher(String, '/car/controller_diagnostic', 10)
        self.create_subscription(
            LaneEstimate, '/car/lane_estimate', self._on_estimate, 10)
        self.create_subscription(
            Odometry, '/car/odometry', self._on_odometry, 10)
        self.create_subscription(
            LeadEstimate, '/car/lead_estimate', self._on_lead_distance, 10)
        self.timer = self.create_timer(
            1.0 / self.config.command_rate_hz, self._on_timer)

    def _on_estimate(self, message: LaneEstimate) -> None:
        self.estimate = message

    def _on_odometry(self, message: Odometry) -> None:
        self.odometry = message

    def _on_lead_distance(self, message: LeadEstimate) -> None:
        self.lead_distance = message.distance if message.detected else None
        self.lead_stamp_s = (
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if message.valid else None)

    def _on_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        estimate_present = self.estimate is not None
        estimate_valid = False
        estimate_stamp_s = 0.0
        target_x_m = 0.0
        target_y_m = 0.0
        if self.estimate is not None:
            estimate_valid = self.estimate.valid
            estimate_stamp_s = (
                self.estimate.header.stamp.sec +
                self.estimate.header.stamp.nanosec * 1e-9)
            target_x_m = self.estimate.lookahead_target.x
            target_y_m = self.estimate.lookahead_target.y

        odom_present = self.odometry is not None
        odom_stamp_s = 0.0
        if self.odometry is not None:
            odom_stamp_s = (
                self.odometry.header.stamp.sec +
                self.odometry.header.stamp.nanosec * 1e-9)

        lead_age_s = None
        if self.lead_stamp_s is not None:
            lead_age_s = now_s - self.lead_stamp_s
        command = self.follower.update(
            now_s, estimate_present, estimate_valid, estimate_stamp_s,
            target_x_m, target_y_m, odom_present, odom_stamp_s,
            self.lead_distance, lead_age_s)
        message = Twist()
        message.linear.x = command.speed_mps
        message.angular.z = command.yaw_rate_radps
        diagnostic = String()
        diagnostic.data = json.dumps({'stamp_s': now_s, 'status': command.status,
                                      'command_speed_mps': command.speed_mps})
        self.diagnostic_publisher.publish(diagnostic)
        self.publisher.publish(message)
        if command.status != self.last_status:
            self.get_logger().info(f'controller state: {command.status}')
            self.last_status = command.status


def main() -> None:
    """Run the lane controller node."""
    rclpy.init()
    node = LaneController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
