"""Keyboard and scripted teleoperation for the car."""

import argparse
import csv
import json
import math
from pathlib import Path
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

CSV_FIELDS = ['t_s', 'x_m', 'y_m', 'yaw_rad', 'vx_mps', 'wz_radps']


def yaw_from_quaternion(quaternion) -> float:
    """Return yaw in radians from a quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z +
                     quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y +
                           quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def parse_segments(text: str) -> list:
    """Parse ``linear,angular,duration;...`` into command segments."""
    segments = []
    for chunk in text.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [part.strip() for part in chunk.split(',')]
        if len(parts) != 3:
            raise ValueError(
                'each segment must be linear,angular,duration')
        try:
            linear, angular, duration = (float(part) for part in parts)
        except ValueError as error:
            raise ValueError(f'invalid segment {chunk!r}') from error
        if duration <= 0.0:
            raise ValueError('segment duration must be positive')
        segments.append((linear, angular, duration))
    if not segments:
        raise ValueError('at least one segment is required')
    return segments


class Teleop(Node):
    """Publish speed and yaw-rate commands and record odometry."""

    def __init__(self, linear_speed: float, angular_speed: float) -> None:
        """Create the teleop publisher with the configured speed limits."""
        super().__init__('teleop')
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.linear = 0.0
        self.angular = 0.0
        self.publisher = self.create_publisher(Twist, '/car/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/car/odometry', self._on_odometry, 10)

        self.samples = 0
        self.distance = 0.0
        self.accumulated_yaw = 0.0
        self.max_speed = 0.0
        self.last_vx = 0.0
        self.first_position = None
        self.last_position = None
        self.last_sample = None
        self.last_yaw = None
        self.first_time = None
        self.last_time = None

        self._record_path = None
        self._record_stream = None
        self._record_writer = None

    def _on_odometry(self, message: Odometry) -> None:
        timestamp = (
            message.header.stamp.sec +
            message.header.stamp.nanosec * 1e-9)
        position = message.pose.pose.position
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        vx = message.twist.twist.linear.x
        wz = message.twist.twist.angular.z

        if self.first_time is None:
            self.first_time = timestamp
            self.first_position = (position.x, position.y, yaw)
        self.last_time = timestamp
        self.last_position = (position.x, position.y, yaw)
        self.samples += 1
        self.max_speed = max(self.max_speed, abs(vx))
        self.last_vx = vx

        if self.last_sample is not None:
            self.distance += math.hypot(
                position.x - self.last_sample[0],
                position.y - self.last_sample[1])
        self.last_sample = (position.x, position.y)

        if self.last_yaw is not None:
            delta = math.atan2(
                math.sin(yaw - self.last_yaw),
                math.cos(yaw - self.last_yaw))
            self.accumulated_yaw += delta
        self.last_yaw = yaw

        if self._record_writer is not None:
            self._record_writer.writerow([
                f'{timestamp:.6f}',
                f'{position.x:.6f}',
                f'{position.y:.6f}',
                f'{yaw:.6f}',
                f'{vx:.6f}',
                f'{wz:.6f}',
            ])

    def _start_recording(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._record_path = path
        self._record_stream = path.open('w', encoding='utf-8', newline='')
        self._record_writer = csv.writer(
            self._record_stream, lineterminator='\n')
        self._record_writer.writerow(CSV_FIELDS)

    def _stop_recording(self) -> None:
        if self._record_stream is not None:
            self._record_stream.close()
            self._record_stream = None
            self._record_writer = None

    def _publish(self) -> None:
        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.publisher.publish(message)

    def _summary(self) -> dict:
        duration = 0.0
        if self.first_time is not None and self.last_time is not None:
            duration = self.last_time - self.first_time
        summary = {
            'samples': self.samples,
            'duration_s': round(duration, 3),
            'distance_m': round(self.distance, 4),
            'heading_change_rad': round(self.accumulated_yaw, 4),
            'max_speed_mps': round(self.max_speed, 4),
            'final_speed_mps': round(self.last_vx, 4),
        }
        if self.first_position is not None:
            summary['start'] = [round(value, 4)
                                for value in self.first_position]
            summary['end'] = [round(value, 4)
                              for value in self.last_position]
        if self._record_path is not None:
            summary['record'] = str(self._record_path)
        return summary


def _read_key(timeout: float):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def _drive_segment(node: Teleop, index: int, segment: tuple) -> None:
    linear, angular, duration = segment
    before_distance = node.distance
    before_yaw = node.accumulated_yaw
    node.linear = linear
    node.angular = angular
    end = time.monotonic() + duration
    while time.monotonic() < end and rclpy.ok():
        node._publish()
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.05)
    print(json.dumps({
        'segment': index,
        'linear_mps': linear,
        'angular_radps': angular,
        'duration_s': duration,
        'distance_m': round(node.distance - before_distance, 4),
        'heading_change_rad': round(
            node.accumulated_yaw - before_yaw, 4),
        'end_speed_mps': round(node.last_vx, 4),
    }))


def _stop_and_settle(node: Teleop, duration: float) -> None:
    node.linear = 0.0
    node.angular = 0.0
    end = time.monotonic() + duration
    while time.monotonic() < end and rclpy.ok():
        node._publish()
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.05)


def _drive_segments(node: Teleop, segments: list, tail: float) -> None:
    for index, segment in enumerate(segments, start=1):
        _drive_segment(node, index, segment)
    _stop_and_settle(node, tail)


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
                node._publish()
                rclpy.spin_once(node, timeout_sec=0.0)
                last = now
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    _stop_and_settle(node, 1.0)
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
    parser.add_argument('--segments', type=str, default='',
                        help='segments as linear,angular,duration;...')
    parser.add_argument('--record', type=str, default='',
                        help='write odometry to this CSV file')
    parser.add_argument('--tail', type=float, default=1.0,
                        help='seconds to record after the last command')
    args = parser.parse_args()

    try:
        if args.segments:
            segments = parse_segments(args.segments)
        elif args.duration > 0.0:
            segments = [(args.linear, args.angular, args.duration)]
        else:
            segments = None
    except ValueError as error:
        print(f'teleop: {error}', file=sys.stderr)
        return 2

    rclpy.init()
    node = Teleop(args.linear, args.angular)
    if args.record:
        node._start_recording(Path(args.record))
    try:
        if segments is not None:
            _drive_segments(node, segments, args.tail)
            return 0
        return _keyboard_loop(node)
    finally:
        node._stop_recording()
        print(json.dumps(node._summary(), indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
