"""Score lane selection and one lane change on a straight two-lane road."""

import argparse
import csv
from pathlib import Path

from auto_drive_sim.evaluator import footprint_corners, yaw_from_quaternion
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from native_runtime import write_json


class LaneProbe(Node):
    def __init__(self, output, center, width, source, target):
        super().__init__('lane_scenario_probe')
        self.output, self.center, self.width = output, center, width
        self.source, self.target = source, target
        self.speed, self.started, self.done = 0.0, None, False
        self.samples = []
        self.image = None
        self.stop_count = 0
        self.bridge = CvBridge()
        self.create_subscription(Twist, '/car/cmd_vel', self.on_speed, 20)
        self.create_subscription(PoseStamped, '/evaluation/ego_pose', self.on_pose, 20)
        self.create_subscription(Image, '/car/camera/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        self.image = msg

    def on_speed(self, msg):
        if self.started is not None and self.speed > 0.05 >= msg.linear.x:
            self.stop_count += 1
            if self.image is not None:
                image = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
                cv2.imwrite(str(self.output / f'stop-{self.stop_count:03d}.png'), image)
        self.speed = msg.linear.x

    def on_pose(self, msg):
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        if now <= 0 or self.done:
            return
        ready = self.output / 'probe-ready.json'
        if not ready.exists():
            write_json(ready, {'ready': True, 'stamp_s': now})
        if self.started is None:
            if self.speed <= 0.05:
                return
            self.started = now
        elapsed = now - self.started
        pose = msg.pose
        x, y = pose.position.x, pose.position.y
        if self.samples and elapsed <= self.samples[-1][0]:
            return
        yaw = yaw_from_quaternion(pose.orientation)
        corners = footprint_corners(x, y, yaw, 3.8, 1.6)
        outer_crossing = any(abs(cy - self.center) > self.width for _, cy in corners)
        source_side = 1 if self.source == 'leftmost' else -1
        within_source = all(source_side * (cy - self.center) >= 0 for _, cy in corners)
        self.samples.append((elapsed, x, y, yaw, self.speed, outer_crossing, within_source))
        if elapsed >= 28.0:
            self.finish()

    def finish(self):
        samples = self.samples
        target_y = self.center + (0.5 if self.target == 'leftmost' else -0.5) * self.width
        tail = [row for row in samples if row[0] >= samples[-1][0] - 3.0]
        max_error = max(abs(row[2] - target_y) for row in tail)
        max_heading_error = max(abs(row[3]) for row in tail)
        centers = [row[2] >= self.center for row in samples]
        divider_crossings = sum(a != b for a, b in zip(centers, centers[1:]))
        outer_crossing = any(row[5] for row in samples)
        changing = self.source != self.target
        stops = sum(a[4] > 0.05 and b[4] <= 0.05 for a, b in zip(samples, samples[1:]))
        complete = len(samples) >= 520 and all(b[0] - a[0] <= 0.1 for a, b in zip(samples, samples[1:]))
        passed = (complete and not outer_crossing and stops == 0 and
                  divider_crossings == int(changing) and
                  (changing or all(row[6] for row in samples)) and
                  max_error <= 0.35 and max_heading_error <= 0.12 and
                  samples[-1][1] - samples[0][1] >= 20)
        result = {
            'pass': passed, 'source_lane': self.source, 'target_lane': self.target,
            'samples': len(samples), 'duration_s': samples[-1][0], 'divider_crossings': divider_crossings,
            'outer_boundary_crossing': outer_crossing, 'commanded_stops': stops,
            'final_window_max_lateral_error_m': max_error,
            'final_window_max_heading_error_rad': max_heading_error,
        }
        with (self.output / 'trajectory.csv').open('w', newline='') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(('elapsed_s', 'x_m', 'y_m', 'yaw_rad', 'command_speed_mps',
                             'outer_crossing', 'within_source_lane'))
            writer.writerows(samples)
        write_json(self.output / 'result.json', result)
        self.done = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('center', type=float)
    parser.add_argument('width', type=float)
    parser.add_argument('source')
    parser.add_argument('target')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LaneProbe(args.output, args.center, args.width, args.source, args.target)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
