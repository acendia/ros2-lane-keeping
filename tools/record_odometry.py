#!/usr/bin/env python3
"""Record /car/odometry to CSV for a fixed simulated duration."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

CSV_FIELDS = ['t_s', 'x_m', 'y_m', 'yaw_rad', 'vx_mps', 'wz_radps']


def _yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z +
                     quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y +
                           quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class Recorder(Node):
    """Write each odometry sample until the requested duration ends."""

    def __init__(self, output: Path, duration_s: float,
                 wall_timeout_s: float) -> None:
        """Open the CSV and subscribe to odometry."""
        super().__init__('odometry_recorder')
        self.output = output
        output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output.open('w', encoding='utf-8', newline='')
        self.writer = csv.writer(self.stream, lineterminator='\n')
        self.writer.writerow(CSV_FIELDS)
        self.duration_s = duration_s
        self.wall_deadline = time.monotonic() + wall_timeout_s
        self.first_stamp = None
        self.last_stamp = None
        self.samples = 0
        self.done = False
        self.create_subscription(
            Odometry, '/car/odometry', self._on_odometry, 10)

    def _on_odometry(self, message: Odometry) -> None:
        if self.done:
            return
        stamp = (
            message.header.stamp.sec +
            message.header.stamp.nanosec * 1e-9)
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp
        position = message.pose.pose.position
        yaw = _yaw_from_quaternion(message.pose.pose.orientation)
        self.writer.writerow([
            f'{stamp:.6f}',
            f'{position.x:.6f}',
            f'{position.y:.6f}',
            f'{yaw:.6f}',
            f'{message.twist.twist.linear.x:.6f}',
            f'{message.twist.twist.angular.z:.6f}',
        ])
        self.samples += 1
        if self.duration_s > 0.0 and stamp - self.first_stamp >= self.duration_s:
            self.done = True

    def close(self) -> dict:
        """Close the CSV and return a summary."""
        if not self.stream.closed:
            self.stream.close()
        duration = 0.0
        if self.first_stamp is not None and self.last_stamp is not None:
            duration = self.last_stamp - self.first_stamp
        return {
            'samples': self.samples,
            'duration_s': round(duration, 3),
            'output': str(self.output),
        }


def main() -> int:
    """Run the recorder."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=Path('artifacts/odometry.csv'))
    parser.add_argument('--duration', type=float, default=30.0,
                        help='simulated seconds to record')
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='wall-clock safety timeout')
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args.output, args.duration, args.timeout)
    try:
        while rclpy.ok() and not node.done:
            if time.monotonic() >= node.wall_deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        summary = node.close()
        print(json.dumps(summary, indent=2), flush=True)
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
