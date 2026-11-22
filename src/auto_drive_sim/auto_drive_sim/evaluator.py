"""Ground-truth evaluator for one lane-following run."""

import csv
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from auto_drive_sim_interfaces.msg import LaneEstimate
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
import yaml

CSV_FIELDS = [
    't_s', 'x_m', 'y_m', 'yaw_rad', 's_m',
    'lateral_error_m', 'cmd_speed_mps', 'perception_valid',
]


def yaw_from_quaternion(quaternion) -> float:
    """Return yaw in radians from a quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z +
                     quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y +
                           quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def nearest_index(x_m: float, y_m: float, center_x: np.ndarray,
                  center_y: np.ndarray) -> int:
    """Return the index of the nearest centerline sample."""
    distances = (center_x - x_m) ** 2 + (center_y - y_m) ** 2
    return int(np.argmin(distances))


def signed_lateral_error(x_m: float, y_m: float, center_x: float,
                         center_y: float, center_yaw: float) -> float:
    """Return the signed lateral offset from a centerline sample."""
    delta_x = x_m - center_x
    delta_y = y_m - center_y
    return delta_x * -math.sin(center_yaw) + delta_y * math.cos(center_yaw)


def footprint_corners(x_m: float, y_m: float, yaw_rad: float,
                      length_m: float, width_m: float) -> list:
    """Return the four footprint corners in the world frame."""
    corners = []
    for forward in (-length_m / 2.0, length_m / 2.0):
        for lateral in (-width_m / 2.0, width_m / 2.0):
            corners.append((
                x_m + forward * math.cos(yaw_rad) -
                lateral * math.sin(yaw_rad),
                y_m + forward * math.sin(yaw_rad) +
                lateral * math.cos(yaw_rad),
            ))
    return corners


def advance_progress(previous_s: float, current_s: float, length_m: float,
                     progress_m: float) -> float:
    """Accumulate signed progress along a closed centerline."""
    delta = current_s - previous_s
    if delta < -length_m / 2.0:
        delta += length_m
    elif delta > length_m / 2.0:
        delta -= length_m
    return progress_m + delta


class Evaluator(Node):
    """Measure lap completion, lateral error, crossings, and stops."""

    def __init__(self) -> None:
        """Load the track and subscribe to ground truth and perception."""
        super().__init__('evaluator')
        self.declare_parameter('ready_file', '')
        self.declare_parameter('scenario', 'center')
        self.declare_parameter('start_x', -10.0)
        self.declare_parameter('start_y', 15.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('output_dir', 'results/center')
        self.declare_parameter('timeout_s', 130.0)
        self.declare_parameter('centerline_path', '')
        self.declare_parameter('track_config_path', '')
        self.declare_parameter('lane_center_y', 15.0)

        share = Path(get_package_share_directory('auto_drive_sim'))
        centerline_path = self.get_parameter('centerline_path').value
        track_path = self.get_parameter('track_config_path').value
        if not centerline_path:
            centerline_path = str(share / 'models/road/centerline.csv')
        if not track_path:
            track_path = str(share / 'config/track.yaml')

        with Path(centerline_path).open('r', encoding='utf-8') as stream:
            samples = list(csv.DictReader(stream))
        self.center_s = np.array([float(row['s_m']) for row in samples])
        self.center_x = np.array([float(row['x_m']) for row in samples])
        self.center_y = np.array([float(row['y_m']) for row in samples])
        self.center_yaw = np.array(
            [float(row['yaw_rad']) for row in samples])
        self.center_length = float(self.center_s[-1]) + 0.1

        with Path(track_path).open('r', encoding='utf-8') as stream:
            track = yaml.safe_load(stream)
        self.lane_width_m = float(track['track']['lane_width_m'])
        self.vehicle_length_m = 3.8
        self.vehicle_width_m = 1.6

        self.scenario = self.get_parameter('scenario').value
        self.start = np.array([
            self.get_parameter('start_x').value,
            self.get_parameter('start_y').value,
            self.get_parameter('start_yaw').value,
        ])
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.timeout_s = self.get_parameter('timeout_s').value

        self.first_time = None
        self.motion_start_time = None
        self.last_time = None
        self.previous_s = None
        self.progress_m = 0.0
        self.samples = []
        self.errors = []
        self.crossing_samples = 0
        self.first_crossing_s = None
        self.perception_messages = 0
        self.perception_valid_messages = 0
        self.latest_perception_valid = False
        self.cmd_speed = 0.0
        self.moving = False
        self.stop_events = 0
        self.stop_times = []
        self.diagnostic_events = []
        self.latest_lane_diagnostic = None
        self.latest_controller_diagnostic = None
        self.latest_image = None
        self.bridge = CvBridge()
        self.lap_complete = False
        self.done = False
        self.wall_start = time.monotonic()

        self.create_subscription(
            PoseStamped, '/evaluation/ego_pose', self._on_model_poses, 10)
        self.create_subscription(
            LaneEstimate, '/car/lane_estimate', self._on_estimate, 10)
        self.create_subscription(
            Twist, '/car/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, '/car/lane_diagnostic', self._on_lane_diagnostic, 10)
        self.create_subscription(String, '/car/controller_diagnostic', self._on_controller_diagnostic, 10)
        self.create_subscription(Image, '/car/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self.create_timer(1.0, self._on_timer)

    def _on_image(self, message: Image) -> None:
        self.latest_image = message

    def _on_lane_diagnostic(self, message: String) -> None:
        value = json.loads(message.data)
        previous = self.latest_lane_diagnostic
        self.latest_lane_diagnostic = value
        if not value['valid'] and (previous is None or previous['valid'] or previous['reason'] != value['reason']):
            self.diagnostic_events.append({'type': 'lane_detection_invalid', **value})

    def _on_controller_diagnostic(self, message: String) -> None:
        value = json.loads(message.data)
        previous = self.latest_controller_diagnostic
        self.latest_controller_diagnostic = value
        if previous is None or previous['status'] != value['status']:
            self.diagnostic_events.append({'type': 'controller_transition', **value})

    def _on_estimate(self, message: LaneEstimate) -> None:
        self.perception_messages += 1
        self.latest_perception_valid = bool(
            message.valid and message.reason == '')
        if self.latest_perception_valid:
            self.perception_valid_messages += 1

    def _on_cmd_vel(self, message: Twist) -> None:
        speed = message.linear.x
        if speed > 0.2:
            self.moving = True
        elif self.moving and speed <= 0.05:
            self.moving = False
            self.stop_events += 1
            now_s = self.get_clock().now().nanoseconds * 1e-9
            self.stop_times.append(round(now_s, 3))
            event = {'type': 'commanded_stop', 'stamp_s': now_s,
                     'controller_observation': self.latest_controller_diagnostic,
                     'lane_observation': self.latest_lane_diagnostic}
            if self.latest_image is not None:
                stamp = self.latest_image.header.stamp
                event['image_stamp_s'] = stamp.sec + stamp.nanosec * 1e-9
                self.output_dir.mkdir(parents=True, exist_ok=True)
                filename = f'stop-{self.stop_events:03d}.png'
                image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                if cv2.imwrite(str(self.output_dir / filename), image):
                    event['image'] = filename
            self.diagnostic_events.append(event)
        self.cmd_speed = speed

    def _on_model_poses(self, message: PoseStamped) -> None:
        if self.done:
            return
        now_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if now_s <= 0.0 or (self.last_time is not None and now_s <= self.last_time):
            return
        pose = message.pose
        x_m = pose.position.x
        y_m = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)
        index = nearest_index(x_m, y_m, self.center_x, self.center_y)
        s_m = float(self.center_s[index])
        error = signed_lateral_error(
            x_m, y_m, float(self.center_x[index]),
            float(self.center_y[index]), float(self.center_yaw[index]))

        if self.first_time is None:
            self.first_time = now_s
            self.previous_s = s_m
        else:
            self.progress_m = advance_progress(
                self.previous_s, s_m, self.center_length, self.progress_m)
            self.previous_s = s_m
        if self.motion_start_time is None and self.progress_m > 0.5:
            self.motion_start_time = now_s
        self.last_time = now_s

        crossed = False
        for corner_x, corner_y in footprint_corners(
                x_m, y_m, yaw, self.vehicle_length_m,
                self.vehicle_width_m):
            corner_index = nearest_index(
                corner_x, corner_y, self.center_x, self.center_y)
            corner_error = signed_lateral_error(
                corner_x, corner_y,
                float(self.center_x[corner_index]),
                float(self.center_y[corner_index]),
                float(self.center_yaw[corner_index]))
            if abs(corner_error) > self.lane_width_m / 2.0:
                crossed = True
                break
        if crossed:
            self.crossing_samples += 1
            if self.first_crossing_s is None:
                self.first_crossing_s = now_s

        self.errors.append(abs(error))
        self.samples.append([
            now_s, x_m, y_m, yaw, s_m, error,
            self.cmd_speed, self.latest_perception_valid,
        ])

        ready_file = self.get_parameter('ready_file').value
        if ready_file and len(self.samples) == 1:
            Path(ready_file).write_text(json.dumps({'ready': True, 'pose_stamp_s': now_s}) + '\n')

        if self.progress_m >= self.center_length:
            self.lap_complete = True
            self._finish()

    def _on_timer(self) -> None:
        if self.done:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.first_time is not None:
            if now_s - self.first_time >= self.timeout_s:
                self._finish()
        elif time.monotonic() - self.wall_start > 40.0:
            self._finish()

    def _finish(self) -> None:
        if self.done:
            return
        self.done = True
        self.output_dir.mkdir(parents=True, exist_ok=True)
        duration = 0.0
        if self.first_time is not None and self.last_time is not None:
            duration = self.last_time - self.first_time
        lap_time = None
        if self.lap_complete and self.motion_start_time is not None:
            lap_time = self.last_time - self.motion_start_time

        errors = np.array(self.errors) if self.errors else np.array([0.0])
        perception_valid_percent = 0.0
        if self.perception_messages > 0:
            perception_valid_percent = (
                100.0 * self.perception_valid_messages /
                self.perception_messages)

        result = {
            'scenario': self.scenario,
            'start_pose': [round(float(value), 3) for value in self.start],
            'start_offset_m': round(
                float(self.start[1] - self.get_parameter('lane_center_y').value), 3),
            'lap_complete': bool(self.lap_complete),
            'lap_time_s': None if lap_time is None else round(lap_time, 3),
            'duration_s': round(duration, 3),
            'progress_m': round(self.progress_m, 3),
            'centerline_length_m': round(self.center_length, 3),
            'lateral_error_mean_abs_m': round(float(errors.mean()), 4),
            'lateral_error_p95_abs_m': round(
                float(np.percentile(errors, 95)), 4),
            'lateral_error_max_abs_m': round(float(errors.max()), 4),
            'lane_width_m': self.lane_width_m,
            'footprint_crossing': self.crossing_samples > 0,
            'footprint_crossing_samples': self.crossing_samples,
            'first_footprint_crossing_s': (
                None if self.first_crossing_s is None
                else round(self.first_crossing_s, 3)),
            'perception_valid_percent': round(perception_valid_percent, 2),
            'perception_messages': self.perception_messages,
            'stop_events': self.stop_events,
            'stop_times_s': self.stop_times,
            'samples': len(self.samples),
        }

        with (self.output_dir / 'lateral_error.csv').open(
                'w', encoding='utf-8', newline='') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(CSV_FIELDS)
            for row in self.samples:
                writer.writerow([
                    f'{row[0]:.6f}', f'{row[1]:.6f}', f'{row[2]:.6f}',
                    f'{row[3]:.6f}', f'{row[4]:.6f}', f'{row[5]:.6f}',
                    f'{row[6]:.6f}', str(row[7]).lower(),
                ])
        (self.output_dir / 'result.json').write_text(
            json.dumps(result, indent=2) + '\n', encoding='utf-8')
        (self.output_dir / 'diagnostic_events.json').write_text(
            json.dumps(self.diagnostic_events, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(result, indent=2), flush=True)


def main() -> None:
    """Run the evaluator until the lap completes or time runs out."""
    rclpy.init()
    node = Evaluator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node._finish()
    except RCLError:
        if rclpy.ok():
            raise
        node._finish()
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
