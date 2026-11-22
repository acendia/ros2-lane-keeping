"""Score longitudinal control using simulation time and stamped observations."""

import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from auto_drive_sim.evaluator import footprint_corners, yaw_from_quaternion
from auto_drive_sim.lane_controller import ControllerConfig
from auto_drive_sim_interfaces.msg import LeadEstimate
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from native_runtime import write_json


def stamp(message):
    time = message.header.stamp
    return time.sec + time.nanosec * 1e-9


def separation(ego, lead, lead_length):
    """Return a signed separating-axis bound between the two footprints."""
    a = np.asarray(footprint_corners(*ego, 3.8, 1.6))
    b = np.asarray(footprint_corners(*lead, lead_length, 1.6))
    gaps = []
    for yaw in (ego[2], lead[2]):
        for axis in ((np.cos(yaw), np.sin(yaw)), (-np.sin(yaw), np.cos(yaw))):
            pa, pb = a @ axis, b @ axis
            gaps.append(max(pb.min() - pa.max(), pa.min() - pb.max()))
    return float(max(gaps))


class LeadProbe(Node):
    def __init__(self, mode, output, world):
        super().__init__('lead_scenario_probe')
        self.mode, self.output = mode, output
        self.ego, self.lead = {}, {}
        self.estimates, self.commands, self.speed = [], [], []
        self.started = None
        self.cut_requested_at = None
        self.ready = False
        self.done = False
        if mode != 'moving':
            model = ET.parse(world).find('./world/model[@name="lead_vehicle"]')
            x, y, _, _, _, yaw = map(float, model.findtext('pose').split())
            self.static_pose = (x, y, yaw)
            self.static_length = float(model.findtext('link/collision/geometry/box/size').split()[0])
        share = Path(get_package_share_directory('auto_drive_sim'))
        self.track = np.loadtxt(share / 'models/road_two_lane/centerline.csv',
                                delimiter=',', skiprows=1, usecols=(1, 2, 3))
        self.create_subscription(PoseStamped, '/evaluation/ego_pose',
                                 lambda msg: self.record_pose(self.ego, msg), 30)
        self.create_subscription(PoseStamped, '/evaluation/lead_pose',
                                 lambda msg: self.record_pose(self.lead, msg), 30)
        self.create_subscription(LeadEstimate, '/car/lead_estimate', self.on_estimate, 30)
        self.create_subscription(String, '/car/controller_diagnostic', self.on_command, 30)
        self.create_subscription(Odometry, '/car/odometry', self.on_speed, 30)
        self.create_timer(0.05, self.tick)

    @staticmethod
    def record_pose(poses, message):
        pose = message.pose
        poses[stamp(message)] = (pose.position.x, pose.position.y,
                                 yaw_from_quaternion(pose.orientation))

    def on_estimate(self, message):
        self.estimates.append({
            'stamp_s': stamp(message), 'valid': message.valid, 'detected': message.detected,
            'distance_m': message.distance if message.valid and message.detected else None,
        })

    def on_speed(self, message):
        self.speed.append((stamp(message), message.twist.twist.linear.x))

    def on_command(self, message):
        command = json.loads(message.data)
        self.commands.append(command)
        if self.started is None and command['command_speed_mps'] > 0:
            self.started = command['stamp_s']

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        detected = self.estimates and self.estimates[-1]['distance_m'] is not None
        if not self.ready and self.ego and detected and (self.lead or self.mode != 'moving'):
            write_json(self.output / 'probe-ready.json', {'ready': True, 'stamp_s': now})
            self.ready = True
        if self.started is None:
            return
        if self.mode == 'loss' and self.cut_requested_at is None and detected and (
                self.commands[-1]['command_speed_mps'] >= 0.3):
            write_json(self.output / 'cut-detector.json', {'stamp_s': now})
            self.cut_requested_at = now
        duration = {'static': 36.0, 'moving': 60.0, 'loss': 10.0}[self.mode]
        if now - self.started >= duration:
            self.finish(now)

    def finish(self, now):
        times = sorted(t for t in self.ego if self.started <= t <= now)
        length = 3.8 if self.mode == 'moving' else self.static_length
        rows = []
        for t in times:
            lead = self.lead.get(t) if self.mode == 'moving' else self.static_pose
            if lead is None:
                continue
            ego = self.ego[t]
            corners = np.asarray(footprint_corners(*ego, 3.8, 1.6))
            nearest = ((corners[:, None, :] - self.track[:, :2]) ** 2).sum(axis=2).argmin(axis=1)
            track = self.track[nearest]
            lateral = (corners - track[:, :2]) * np.column_stack((-np.sin(track[:, 2]), np.cos(track[:, 2])))
            crossing = bool((np.abs(lateral.sum(axis=1)) > 1.75).any())
            rows.append((t, *ego, *lead, separation(ego, lead, length), crossing))
        series_complete = bool(rows and rows[0][0] - self.started <= 0.1 and
                               now - rows[-1][0] <= 0.1 and
                               max(np.diff([row[0] for row in rows]), default=1.0) <= 0.1)
        gap = min((row[-2] for row in rows), default=None)
        crossing = any(row[-1] for row in rows)
        final_speed = [abs(speed) for t, speed in self.speed if now - 2 <= t <= now]
        settled = len(final_speed) >= 30 and max(final_speed) <= 0.05
        last = self.estimates[-1] if self.estimates else {}
        stale = next((c for c in self.commands if c['stamp_s'] >= self.started and
                      c['status'] == 'stale_lead_estimate'), None)
        delay = (stale['stamp_s'] - stale['lead_stamp_s']
                 if stale and stale['lead_stamp_s'] is not None else None)
        final_camera = last.get('distance_m')
        final_camera_fresh = final_camera is not None and 0 <= now - last['stamp_s'] <= 0.3
        tail_commands = [c for c in self.commands if c['stamp_s'] >= now - 2]
        stopped = bool(tail_commands) and all(c['command_speed_mps'] == 0 for c in tail_commands)
        final_state = tail_commands[-1]['status'] if tail_commands else None
        travel = sum(np.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(rows, rows[1:]))
        config = ControllerConfig()
        following_distances = [e['distance_m'] for e in self.estimates
                               if now - 10 <= e['stamp_s'] <= now and e['distance_m'] is not None]
        expected_range = config.lead_follow_gap_m + 1.0 / config.lead_speed_gain
        if self.mode == 'static':
            case_pass = (settled and stopped and final_state == 'lead_vehicle' and
                         final_camera_fresh and abs(final_camera - config.lead_follow_gap_m) <= 0.25)
        elif self.mode == 'moving':
            case_pass = (travel >= 40.0 and stale is None and final_camera_fresh and
                         len(following_distances) >= 120 and
                         all(abs(distance - expected_range) <= 0.75 for distance in following_distances) and
                         len(final_speed) >= 30 and all(0.8 <= speed <= 1.2 for speed in final_speed))
        else:
            case_pass = (self.cut_requested_at is not None and settled and stopped and
                         final_state == 'stale_lead_estimate' and delay is not None and
                         stale['stamp_s'] >= self.cut_requested_at and
                         0.0 <= delay <= config.lead_timeout_s + 1 / config.command_rate_hz + 1e-6 and
                         stale['command_speed_mps'] == 0)
        result = {
            'scenario': self.mode, 'pass': bool(series_complete and gap > 0 and not crossing and case_pass),
            'duration_s': now - self.started, 'pose_samples': len(rows),
            'pose_series_complete': series_complete, 'minimum_footprint_separation_bound_m': gap,
            'footprint_crossing': crossing, 'ego_travel_m': travel, 'settled': settled,
            'final_camera_distance_m': final_camera if final_camera_fresh else None,
            'last_10s_camera_range_m': ([min(following_distances), max(following_distances)]
                                        if following_distances else None),
            'stale_stop_delay_s': delay, 'final_state': final_state,
            'max_final_2s_speed_mps': max(final_speed, default=None),
        }
        with (self.output / 'trajectory.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('stamp_s', 'ego_x_m', 'ego_y_m', 'ego_yaw_rad', 'lead_x_m',
                             'lead_y_m', 'lead_yaw_rad', 'separation_bound_m', 'footprint_crossing'))
            writer.writerows(rows)
        write_json(self.output / 'telemetry.json', {
            'lead_estimates': self.estimates, 'commands': self.commands, 'actual_speed_mps': self.speed,
        })
        write_json(self.output / 'result.json', result)
        self.done = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('static', 'moving', 'loss'))
    parser.add_argument('output', type=Path)
    parser.add_argument('world', type=Path)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LeadProbe(args.mode, args.output, args.world)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
