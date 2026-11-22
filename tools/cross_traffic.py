"""Script the northbound simulator vehicle for the yielding scenario."""

import json

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from auto_drive_sim.intersection import stamp_s


class CrossTraffic(Node):
    def __init__(self):
        super().__init__('cross_traffic')
        self.released = False
        self.odom = None
        self.publisher = self.create_publisher(Twist, '/lead/cmd_vel', 10)
        self.create_subscription(Odometry, '/lead/odometry', self.on_odometry, 10)
        self.create_subscription(String, '/car/intersection_state', self.on_state, 10)
        self.create_timer(0.05, self.tick)

    def on_odometry(self, msg):
        self.odom = msg

    def on_state(self, msg):
        state = json.loads(msg.data)
        if state['phase'] == 'hold' and state['actual_speed_mps'] <= 0.05:
            self.released = True

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        command = Twist()
        if self.released and self.odom and 0 <= now - stamp_s(self.odom) <= 0.3:
            remaining = 16.0 - self.odom.pose.pose.position.x
            command.linear.x = min(1.8, max(0.0, remaining))
        self.publisher.publish(command)


if __name__ == '__main__':
    rclpy.init()
    node = CrossTraffic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
