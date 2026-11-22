#!/usr/bin/env python3
"""Measure image freshness in simulation time, independently of wall FPS."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--topic', default='/camera_check/image')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('camera_timing_probe', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    rows = []

    def receive(message):
        rows.append((time.monotonic(),
                     message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                     node.get_clock().now().nanoseconds * 1e-9))

    node.create_subscription(Image, args.topic, receive, qos_profile_sensor_data)
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = {'frames': len(rows), 'requested_wall_duration_s': args.duration}
    if len(rows) > 1:
        values = np.array(rows)
        wall_span = values[-1, 0] - values[0, 0]
        sim_span = values[-1, 1] - values[0, 1]
        gaps = np.diff(values[:, 1])
        ages = values[:, 2] - values[:, 1]
        result.update(
            wall_span_s=wall_span, simulation_span_s=sim_span,
            wall_rate_hz=(len(rows) - 1) / wall_span,
            sim_rate_hz=(len(rows) - 1) / sim_span if sim_span > 0 else 0,
            observed_real_time_factor=sim_span / wall_span,
            max_sim_gap_s=float(gaps.max()),
            gaps_over_0_3_s=int((gaps > 0.3).sum()),
            duplicate_or_reversed_stamps=int((gaps <= 0).sum()),
            arrival_age_p95_s=float(np.percentile(ages, 95)),
            arrival_age_max_s=float(ages.max()))
    result['timing_pass'] = bool(
        len(rows) > 1 and result['gaps_over_0_3_s'] == 0 and
        result['duplicate_or_reversed_stamps'] == 0 and
        result['arrival_age_max_s'] <= 0.3 and result['sim_rate_hz'] >= 14)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['timing_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
