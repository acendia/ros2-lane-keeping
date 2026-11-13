"""Pure-pursuit lane following controller."""

from dataclasses import dataclass
import math
from typing import Optional

from auto_drive_sim_interfaces.msg import LaneEstimate
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


@dataclass(frozen=True)
class ControllerConfig:
    """Controller limits and smoothing."""

    target_speed_mps: float = 1.5
    max_acceleration_mps2: float = 0.6
    max_curvature_1pm: float = 0.15
    max_yaw_rate_radps: float = 0.6
    lookahead_gain: float = 1.0
    yaw_smoothing_alpha: float = 0.35
    max_lane_age_s: float = 0.3
    max_odom_age_s: float = 0.3
    command_rate_hz: float = 20.0

    def __post_init__(self) -> None:
        """Validate the parameter ranges."""
        if self.target_speed_mps <= 0.0:
            raise ValueError('target_speed_mps must be positive')
        if self.max_acceleration_mps2 <= 0.0:
            raise ValueError('max_acceleration_mps2 must be positive')
        if self.max_curvature_1pm <= 0.0:
            raise ValueError('max_curvature_1pm must be positive')
        if self.max_yaw_rate_radps <= 0.0:
            raise ValueError('max_yaw_rate_radps must be positive')
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


class LaneFollower:
    """Stateful speed ramp, pure-pursuit curvature, and yaw smoothing."""

    def __init__(self, config: ControllerConfig) -> None:
        """Create a follower with zero speed and yaw."""
        self.config = config
        self.speed_mps = 0.0
        self.yaw_rate_radps = 0.0
        self.last_update_s = None

    def update(self, now_s: float, estimate_present: bool,
               estimate_valid: bool, estimate_stamp_s: float,
               target_x_m: float, target_y_m: float,
               odom_present: bool, odom_stamp_s: float) -> ControllerCommand:
        """Update the controller and return the current command."""
        dt = 0.0
        if self.last_update_s is not None:
            dt = max(0.0, min(0.2, now_s - self.last_update_s))
        self.last_update_s = now_s

        reason = self._stop_reason(
            now_s, estimate_present, estimate_valid, estimate_stamp_s,
            odom_present, odom_stamp_s)
        if reason:
            self.speed_mps = 0.0
            self.yaw_rate_radps = 0.0
            return ControllerCommand(0.0, 0.0, reason)

        lookahead_x = target_x_m * self.config.lookahead_gain
        lookahead_y = target_y_m * self.config.lookahead_gain
        curvature = curvature_for_point(
            lookahead_x, lookahead_y, self.config.max_curvature_1pm)
        if curvature is None:
            self.speed_mps = 0.0
            self.yaw_rate_radps = 0.0
            return ControllerCommand(0.0, 0.0, 'bad_lookahead')

        self.speed_mps = min(
            self.config.target_speed_mps,
            self.speed_mps + self.config.max_acceleration_mps2 * dt)
        target_yaw = self.speed_mps * curvature
        target_yaw = max(-self.config.max_yaw_rate_radps,
                         min(self.config.max_yaw_rate_radps, target_yaw))
        alpha = self.config.yaw_smoothing_alpha
        self.yaw_rate_radps = (
            alpha * target_yaw + (1.0 - alpha) * self.yaw_rate_radps)
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
        self.declare_parameter('max_curvature_1pm', 0.15)
        self.declare_parameter('max_yaw_rate_radps', 0.6)
        self.declare_parameter('lookahead_gain', 1.0)
        self.declare_parameter('yaw_smoothing_alpha', 0.35)
        self.declare_parameter('max_lane_age_s', 0.3)
        self.declare_parameter('max_odom_age_s', 0.3)
        self.declare_parameter('command_rate_hz', 20.0)

        self.config = ControllerConfig(
            target_speed_mps=self.get_parameter(
                'target_speed_mps').value,
            max_acceleration_mps2=self.get_parameter(
                'max_acceleration_mps2').value,
            max_curvature_1pm=self.get_parameter(
                'max_curvature_1pm').value,
            max_yaw_rate_radps=self.get_parameter(
                'max_yaw_rate_radps').value,
            lookahead_gain=self.get_parameter(
                'lookahead_gain').value,
            yaw_smoothing_alpha=self.get_parameter(
                'yaw_smoothing_alpha').value,
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
        self.last_status = ''
        self.publisher = self.create_publisher(Twist, '/car/cmd_vel', 10)
        self.create_subscription(
            LaneEstimate, '/car/lane_estimate', self._on_estimate, 10)
        self.create_subscription(
            Odometry, '/car/odometry', self._on_odometry, 10)
        self.timer = self.create_timer(
            1.0 / self.config.command_rate_hz, self._on_timer)

    def _on_estimate(self, message: LaneEstimate) -> None:
        self.estimate = message

    def _on_odometry(self, message: Odometry) -> None:
        self.odometry = message

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

        command = self.follower.update(
            now_s, estimate_present, estimate_valid, estimate_stamp_s,
            target_x_m, target_y_m, odom_present, odom_stamp_s)
        message = Twist()
        message.linear.x = command.speed_mps
        message.angular.z = command.yaw_rate_radps
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
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
