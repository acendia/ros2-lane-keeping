#!/usr/bin/env python3
"""Measure a ROS camera stream and save one frame as evidence.

This probe is part of the Milestone 0 gate. It does not assume the image is
valid: it checks dimensions, frame count, and pixel contrast, then exits
non-zero when the stream is missing or blank.
"""

import argparse
import json
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraProbe(Node):
    """Subscribe to an image topic and retain the latest frame and timing."""

    def __init__(self, topic: str) -> None:
        """Create a probe subscribed to the given image topic."""
        super().__init__('camera_probe')
        self._bridge = CvBridge()
        self.frames = 0
        self.first_wall = None
        self.last_wall = None
        self.first_stamp = None
        self.last_stamp = None
        self.last_image = None
        self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self.first_wall is None:
            self.first_wall = now
        if self.first_stamp is None:
            self.first_stamp = msg.header.stamp
        self.last_wall = now
        self.last_stamp = msg.header.stamp
        self.last_image = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self.frames += 1


def stamp_seconds(stamp) -> float:
    """Convert a builtin_interfaces Time stamp to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def main() -> int:
    """Run the probe, print a JSON summary, and return a process status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/camera_check/image')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--output', default='artifacts/camera_check/frame.png')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--min-frames', type=int, default=10)
    args = parser.parse_args()

    rclpy.init()
    node = CameraProbe(args.topic)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    summary = {
        'topic': args.topic,
        'frames': node.frames,
        'requested_duration_s': args.duration,
    }

    if node.frames < 2 or node.last_image is None:
        summary['valid'] = False
        summary['reason'] = 'no image frames received'
        print(json.dumps(summary, indent=2))
        return 1

    image = node.last_image
    height, width = image.shape[:2]
    mean, std = cv2.meanStdDev(image)
    wall_span = max(node.last_wall - node.first_wall, 1e-9)
    stamp_span = max(
        stamp_seconds(node.last_stamp) - stamp_seconds(node.first_stamp), 1e-9)

    summary.update({
        'width': width,
        'height': height,
        'mean_bgr': [round(float(v), 2) for v in mean.flatten()],
        'std_bgr': [round(float(v), 2) for v in std.flatten()],
        'wall_span_s': round(wall_span, 3),
        'stamp_span_s': round(stamp_span, 3),
        'wall_rate_hz': round((node.frames - 1) / wall_span, 2),
        'stamp_rate_hz': round((node.frames - 1) / stamp_span, 2),
        'output': args.output,
    })

    problems = []
    if width != args.width or height != args.height:
        problems.append(
            f'expected {args.width}x{args.height}, got {width}x{height}')
    if node.frames < args.min_frames:
        problems.append(
            f'only {node.frames} frames in {args.duration:.1f} s')
    if float(std.max()) < 5.0:
        problems.append('image is nearly uniform (std < 5)')

    summary['valid'] = not problems
    if problems:
        summary['reason'] = '; '.join(problems)

    cv2.imwrite(args.output, image)
    print(json.dumps(summary, indent=2))
    return 0 if summary['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
