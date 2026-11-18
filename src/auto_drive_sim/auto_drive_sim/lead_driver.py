"""Drive the lead car along the right-lane centerline."""

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def yaw_from_quaternion(quaternion) -> float:
    """Return yaw in radians from a quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z +
                     quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y +
                           quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class LeadDriver(Node):
    """Follow the generated right-lane centerline at a fixed speed."""

    def __init__(self) -> None:
        """Load the centerline and create the odometry subscription."""
        super().__init__('lead_driver')
        self.declare_parameter('target_speed_mps', 1.0)
        self.declare_parameter('lookahead_m', 4.0)
        self.declare_parameter('start_x', 5.0)
        self.declare_parameter('start_y', 13.19)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('max_curvature_1pm', 0.15)
        self.declare_parameter('max_yaw_rate_radps', 0.6)
        self.target_speed = float(
            self.get_parameter('target_speed_mps').value)
        self.lookahead = float(self.get_parameter('lookahead_m').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_yaw = float(self.get_parameter('start_yaw').value)
        self.max_curvature = float(
            self.get_parameter('max_curvature_1pm').value)
        self.max_yaw_rate = float(
            self.get_parameter('max_yaw_rate_radps').value)

        share = Path(get_package_share_directory('auto_drive_sim'))
        with (share / 'models/road_two_lane/centerline.csv').open(
                'r', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        self.center_x = [float(row['x_m']) for row in rows]
        self.center_y = [float(row['y_m']) for row in rows]
        self.sample_spacing = (
            float(rows[1]['s_m']) - float(rows[0]['s_m']))
        self.lookahead_steps = max(
            1, int(round(self.lookahead / self.sample_spacing)))
        self.odometry = None
        self.odometry_stamp_s = None
        self.publisher = self.create_publisher(Twist, '/lead/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/lead/odometry', self._on_odometry, 10)
        self.create_timer(0.05, self._on_timer)

    def _on_odometry(self, message: Odometry) -> None:
        self.odometry = message
        self.odometry_stamp_s = (
            message.header.stamp.sec +
            message.header.stamp.nanosec * 1e-9)

    def _on_timer(self) -> None:
        command = Twist()
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.odometry is None or self.odometry_stamp_s is None or \
                now_s - self.odometry_stamp_s > 0.3:
            self.publisher.publish(command)
            return

        pose = self.odometry.pose.pose
        x_m = self.start_x + pose.position.x
        y_m = self.start_y + pose.position.y
        yaw = self.start_yaw + yaw_from_quaternion(pose.orientation)

        nearest = min(
            range(len(self.center_x)),
            key=lambda index: (
                (self.center_x[index] - x_m) ** 2 +
                (self.center_y[index] - y_m) ** 2))
        target_index = (nearest + self.lookahead_steps) % len(self.center_x)
        delta_x = self.center_x[target_index] - x_m
        delta_y = self.center_y[target_index] - y_m
        forward = math.cos(yaw) * delta_x + math.sin(yaw) * delta_y
        left = -math.sin(yaw) * delta_x + math.cos(yaw) * delta_y
        distance_sq = forward * forward + left * left
        if forward <= 0.1 or distance_sq <= 1e-6:
            self.publisher.publish(command)
            return
        curvature = 2.0 * left / distance_sq
        curvature = max(-self.max_curvature,
                        min(self.max_curvature, curvature))
        yaw_rate = max(-self.max_yaw_rate,
                       min(self.max_yaw_rate,
                           self.target_speed * curvature))
        command.linear.x = self.target_speed
        command.angular.z = yaw_rate
        self.publisher.publish(command)


def main() -> None:
    """Run the lead driver node."""
    rclpy.init()
    node = LeadDriver()
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
