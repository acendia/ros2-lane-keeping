"""Camera stop-line detection and speed control for a straight crossing."""

import json
import math

from auto_drive_sim.lane_perception import CameraModel, GroundPlane
from auto_drive_sim.lead_detector import LeadDetectorConfig, estimate_lead_distance
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def stamp_s(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def stop_line_distance(image, ground):
    """Find a wide transverse white stripe in the center of the lane."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, 170), (180, 60, 255))
    half = white.shape[1] // 2
    # A transverse stripe is several neighboring rows of contiguous paint.
    # Painted lane boundaries do not run across the center column.
    depths = []
    for row in np.flatnonzero(white[:, half]):
        dark = np.flatnonzero(white[row] == 0)
        before, after = dark[dark < half], dark[dark > half]
        left = int(before[-1] + 1) if before.size else 0
        right = int(after[0] - 1) if after.size else white.shape[1] - 1
        if left == 0 or right == white.shape[1] - 1:
            continue
        pixels = ((row, left), (row, half), (row, right))
        # Use the same calibrated ground mapping as lane perception.
        (x, y) = ground.points_for_pixels(pixels)
        if len(x) == 3 and 1.2 <= x[1] <= 9.0 and 1.9 <= y[0] - y[2] <= 3.1:
            depths.append(x[1])
    if len(depths) < 2 or max(depths) - min(depths) > 0.7:
        return None
    return float(np.median(depths))


class StraightRoute:
    """Stop at the confirmed line, hold at rest, then continue to the goal."""

    def __init__(self, goal_m, search_limit_m, *, yield_to_traffic=False):
        if not (math.isfinite(goal_m) and 0 < search_limit_m < goal_m):
            raise ValueError('Route requires 0 < search limit < goal')
        self.goal_m = goal_m
        self.search_limit_m = search_limit_m
        self.line_m = None
        self.candidates = []
        self.phase = 'approach'
        self.rest_since = None
        self.yield_to_traffic = yield_to_traffic
        self.saw_traffic = False
        self.clear_since = None

    def observe_line(self, station_m):
        if self.line_m is not None or self.phase != 'approach':
            return
        if station_m is None or not math.isfinite(station_m):
            self.candidates.clear()
            return
        if self.candidates and abs(station_m - self.candidates[0]) > 0.35:
            self.candidates.clear()
        self.candidates.append(station_m)
        if len(self.candidates) == 3:
            self.line_m = float(np.median(self.candidates))

    def limit(self, now, position, speed, fresh, traffic_blocked=None):
        if not fresh:
            self.rest_since = None
            self.clear_since = None
            return 0.0
        if self.phase == 'approach':
            target = self.search_limit_m if self.line_m is None else self.line_m - 2.4
            if self.line_m is None:
                return self.braking_limit(target - position)
            if target - position > 0.12:
                return self.braking_limit(target - position)
            self.phase = 'hold'
        if self.phase == 'hold':
            if traffic_blocked is True:
                self.saw_traffic = True
            if traffic_blocked is False and self.saw_traffic:
                if self.clear_since is None:
                    self.clear_since = now
            else:
                self.clear_since = None
            can_cross = not self.yield_to_traffic or (
                self.clear_since is not None and now - self.clear_since >= 0.8)
            if speed > 0.05:
                self.rest_since = None
            elif self.rest_since is None:
                self.rest_since = now
            elif now - self.rest_since >= 2.0 and can_cross:
                self.phase = 'cross'
            return 0.0
        if self.phase == 'cross':
            if self.goal_m - position > 0.12:
                return self.braking_limit(self.goal_m - position)
            if speed <= 0.05:
                self.phase = 'arrived'
        return 0.0

    @staticmethod
    def braking_limit(remaining):
        return math.sqrt(2 * 0.45 * max(0.0, remaining - 0.10))


class IntersectionController(Node):
    def __init__(self):
        super().__init__('intersection_controller')
        self.declare_parameter('goal_m', 28.0)
        self.declare_parameter('search_limit_m', 6.0)
        self.declare_parameter('yield_to_traffic', False)
        self.route = StraightRoute(self.get_parameter('goal_m').value,
                                   self.get_parameter('search_limit_m').value,
                                   yield_to_traffic=self.get_parameter('yield_to_traffic').value)
        self.camera = CameraModel()
        self.ground = GroundPlane(self.camera)
        self.bridge = CvBridge()
        self.frame_stamp = None
        self.traffic_blocked = None
        self.traffic_config = LeadDetectorConfig(max_distance_m=12.0)
        self.odom = None
        self.lane_command = None
        self.lane_command_stamp = None
        self.previous_speed = 0.0
        self.previous_tick = None
        self.publisher = self.create_publisher(Twist, '/car/cmd_vel', 10)
        self.diagnostics = self.create_publisher(String, '/car/intersection_state', 10)
        self.create_subscription(Image, '/car/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/car/odometry', self.on_odometry, 10)
        self.create_subscription(Twist, '/car/lane_cmd_vel', self.on_lane_command, 10)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odometry(self, message):
        self.odom = message

    def on_lane_command(self, message):
        self.lane_command, self.lane_command_stamp = message, self.now()

    def on_image(self, message):
        stamp = stamp_s(message)
        if not 0 <= self.now() - stamp <= 0.3 or (self.frame_stamp is not None and stamp <= self.frame_stamp):
            return
        self.frame_stamp = stamp
        if self.route.yield_to_traffic and self.route.phase in ('approach', 'hold'):
            image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
            self.traffic_blocked = estimate_lead_distance(
                image, self.ground, self.traffic_config) is not None
        if self.route.line_m is not None or self.odom is None or abs(stamp - stamp_s(self.odom)) > 0.10:
            return
        distance = stop_line_distance(self.bridge.imgmsg_to_cv2(message, 'bgr8'), self.ground)
        station = None
        if distance is not None:
            station = self.odom.pose.pose.position.x + self.camera.x_offset_m + distance
        self.route.observe_line(station)

    def tick(self):
        now = self.now()
        dt = 0.0 if self.previous_tick is None else max(0.0, min(0.1, now - self.previous_tick))
        self.previous_tick = now
        stamps = (self.frame_stamp, stamp_s(self.odom) if self.odom else None, self.lane_command_stamp)
        fresh = all(t is not None and 0 <= now - t <= 0.3 for t in stamps)
        position = self.odom.pose.pose.position.x if self.odom else 0.0
        speed = abs(self.odom.twist.twist.linear.x) if self.odom else 0.0
        cap = self.route.limit(now, position, speed, fresh, self.traffic_blocked)
        command = Twist()
        if cap > 0 and self.lane_command.linear.x > 0:
            command.linear.x = min(cap, self.lane_command.linear.x, self.previous_speed + 0.6 * dt)
            command.angular.z = self.lane_command.angular.z * command.linear.x / self.lane_command.linear.x
        self.previous_speed = command.linear.x
        self.publisher.publish(command)
        self.diagnostics.publish(String(data=json.dumps({
            'stamp_s': now, 'phase': self.route.phase, 'line_station_m': self.route.line_m,
            'actual_speed_mps': speed, 'command_speed_mps': command.linear.x,
            'traffic_blocked': self.traffic_blocked,
        })))


def main():
    rclpy.init()
    node = IntersectionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
