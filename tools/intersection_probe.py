"""Score an empty intersection crossing and the missing-stop-line case."""

import argparse
import csv
import json
from pathlib import Path

from auto_drive_sim.evaluator import footprint_corners, yaw_from_quaternion
from auto_drive_sim.intersection import stamp_s
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from native_runtime import write_json


class IntersectionProbe(Node):
    def __init__(self, args):
        super().__init__('intersection_probe')
        self.args = args
        self.state = None
        self.start = None
        self.samples = []
        self.done = False
        self.create_subscription(String, '/car/intersection_state', self.on_state, 20)
        self.create_subscription(PoseStamped, '/evaluation/ego_pose', self.on_pose, 20)

    def on_state(self, message):
        self.state = json.loads(message.data)

    def on_pose(self, message):
        if self.done or stamp_s(message) <= 0:
            return
        ready = self.args.output / 'probe-ready.json'
        if not ready.exists():
            write_json(ready, {'ready': True})
        if self.state is None or abs(stamp_s(message) - self.state['stamp_s']) > 0.15:
            return
        if self.start is None:
            if self.state['command_speed_mps'] <= 0:
                return
            self.start = stamp_s(message)
        elapsed = stamp_s(message) - self.start
        if self.samples and elapsed <= self.samples[-1][0]:
            return
        pose = message.pose
        x, y = pose.position.x, pose.position.y
        corners = footprint_corners(x, y, yaw_from_quaternion(pose.orientation), 3.8, 1.6)
        crossing = any(abs(corner_y - self.args.lane_y) > self.args.lane_half_width
                       for _, corner_y in corners)
        self.samples.append((elapsed, x, y, max(cx for cx, _ in corners),
                             self.state['phase'], self.state['actual_speed_mps'],
                             self.state['command_speed_mps'], self.state['line_station_m'], crossing))
        if elapsed >= (18.0 if self.args.missing_line else 42.0):
            self.finish()

    def finish(self):
        rows = self.samples
        first_cross = next((i for i, row in enumerate(rows) if row[4] == 'cross'), None)
        pre_cross = rows if first_cross is None else rows[:first_cross]
        stopped = []
        for row in pre_cross:
            if row[4] == 'hold' and row[5] <= 0.05 and row[6] == 0:
                stopped.append(row)
            else:
                stopped.clear()
        dwell = stopped[-1][0] - stopped[0][0] if stopped else 0.0
        clearance = self.args.line_x - stopped[-1][3] if stopped else None
        tail = [row for row in rows if row[0] >= rows[-1][0] - 1.0]
        settled = len(tail) >= 15 and all(row[5] <= 0.05 and row[6] == 0 for row in tail)
        edge = any(row[8] for row in rows)
        early_crossing = any(row[3] >= self.args.line_x for row in pre_cross)
        complete = all(b[0] - a[0] <= 0.15 for a, b in zip(rows, rows[1:]))
        goal_error = abs(rows[-1][1] - self.args.goal_x)
        if self.args.missing_line:
            passed = (first_cross is None and all(row[7] is None for row in rows) and settled and
                      0.0 <= self.args.search_x - rows[-1][1] <= 0.25)
        else:
            passed = (first_cross is not None and dwell >= 1.90 and clearance is not None and
                      0.08 <= clearance <= 0.70 and goal_error <= 0.25 and
                      rows[-1][4] == 'arrived' and settled)
        result = {
            'pass': passed and complete and not edge and not early_crossing,
            'duration_s': rows[-1][0], 'samples': len(rows), 'final_phase': rows[-1][4],
            'stopped_before_line_s': dwell, 'front_clearance_m': clearance,
            'goal_error_m': goal_error, 'final_position_x_m': rows[-1][1],
            'settled': settled, 'pose_series_complete': complete,
            'premature_line_crossing': early_crossing, 'lane_boundary_crossing': edge,
        }
        with (self.args.output / 'trajectory.csv').open('w', newline='') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(('elapsed_s', 'x_m', 'y_m', 'front_x_m', 'phase', 'speed_mps',
                             'command_speed_mps', 'line_station_m', 'lane_crossing'))
            writer.writerows(rows)
        write_json(self.args.output / 'result.json', result)
        self.done = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    for field in ('lane-y', 'lane-half-width', 'line-x', 'goal-x', 'search-x'):
        parser.add_argument(f'--{field}', type=float, required=True)
    parser.add_argument('--missing-line', action='store_true')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = IntersectionProbe(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
