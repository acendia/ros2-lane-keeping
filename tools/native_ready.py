#!/usr/bin/env python3
"""Acknowledge advancing clock and fresh data, not just advertised topics."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def fresh(stamp, clock):
    return stamp > 0 and -0.05 <= clock - stamp <= 0.3


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['camera', 'lane'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=45)
    args = parser.parse_args()
    rclpy.init()
    node = Node('native_readiness', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'clock': 0.0, 'clock_advances': 0, 'image': 0.0, 'lane': 0.0, 'odom': 0.0}

    def clock(message):
        value = seconds(message.clock)
        if value > state['clock']:
            state['clock_advances'] += 1
        state['clock'] = value

    def image(message):
        if message.width == 640 and message.height == 360 and len(message.data):
            state['image'] = seconds(message.header.stamp)

    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(Image, '/camera_check/image' if args.mode == 'camera'
                             else '/car/camera/image_raw', image, qos_profile_sensor_data)
    if args.mode == 'lane':
        from auto_drive_sim_interfaces.msg import LaneEstimate

        def lane(message):
            state['lane'] = seconds(message.header.stamp) if message.valid and message.reason == '' else 0.0

        def odom(message):
            state['odom'] = seconds(message.header.stamp)

        node.create_subscription(LaneEstimate, '/car/lane_estimate', lane, 10)
        node.create_subscription(Odometry, '/car/odometry', odom, 10)
    started = time.monotonic()
    ready = False
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            fields = ('image',) if args.mode == 'camera' else ('image', 'lane', 'odom')
            if state['clock_advances'] >= 3 and all(fresh(state[name], state['clock']) for name in fields):
                ready = True
                break
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    result = dict(state, ready=ready, startup_wall_s=time.monotonic() - started)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    if not ready:
        print(f'Readiness failed: no advancing clock and fresh {args.mode} data within {args.timeout:g}s')
    return 0 if ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
