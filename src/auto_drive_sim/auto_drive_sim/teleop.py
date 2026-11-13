"""Keyboard and scripted teleoperation for the car."""

import argparse
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class Teleop(Node):
    """Publish speed and yaw-rate commands to the car."""

    def __init__(self, linear_speed: float, angular_speed: float) -> None:
        """Create the teleop publisher with the configured speed limits."""
        super().__init__('teleop')
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.linear = 0.0
        self.angular = 0.0
        self.publisher = self.create_publisher(Twist, '/car/cmd_vel', 10)

    def publish(self) -> None:
        """Publish the current command."""
        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.publisher.publish(message)


def _read_key(timeout: float):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def _keyboard_loop(node: Teleop) -> int:
    if not sys.stdin.isatty():
        print('teleop: stdin is not a terminal; use --duration for scripted mode',
              file=sys.stderr)
        return 1

    settings = termios.tcgetattr(sys.stdin)
    print('Teleop: w forward, s reverse, a left, d right, space stop, q quit')
    try:
        tty.setcbreak(sys.stdin.fileno())
        last = time.monotonic()
        while rclpy.ok():
            key = _read_key(0.05)
            if key is not None:
                if key == 'w':
                    node.linear = node.linear_speed
                elif key == 's':
                    node.linear = -node.linear_speed
                elif key == 'a':
                    node.angular = node.angular_speed
                elif key == 'd':
                    node.angular = -node.angular_speed
                elif key == ' ':
                    node.linear = 0.0
                    node.angular = 0.0
                elif key == 'q':
                    break
            now = time.monotonic()
            if now - last >= 1.0 / 20.0:
                node.publish()
                last = now
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    node.linear = 0.0
    node.angular = 0.0
    for _ in range(10):
        node.publish()
        time.sleep(0.02)
    return 0


def _scripted_loop(node: Teleop, linear: float, angular: float,
                   duration: float) -> int:
    node.linear = linear
    node.angular = angular
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and rclpy.ok():
        node.publish()
        time.sleep(1.0 / 20.0)
    node.linear = 0.0
    node.angular = 0.0
    for _ in range(10):
        node.publish()
        time.sleep(0.02)
    return 0


def main() -> int:
    """Run keyboard or scripted teleoperation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--linear', type=float, default=1.5,
                        help='linear speed in m/s')
    parser.add_argument('--angular', type=float, default=0.6,
                        help='yaw rate in rad/s')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='run a fixed command for this many seconds')
    args = parser.parse_args()

    rclpy.init()
    node = Teleop(args.linear, args.angular)
    try:
        if args.duration > 0.0:
            return _scripted_loop(node, args.linear, args.angular,
                                  args.duration)
        return _keyboard_loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
