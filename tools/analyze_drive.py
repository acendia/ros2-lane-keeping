#!/usr/bin/env python3
"""Compare a recorded drive with the generated track centerline."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CENTERLINE = REPO_ROOT / 'src/auto_drive_sim/models/road/centerline.csv'
DEFAULT_TRACK = REPO_ROOT / 'src/auto_drive_sim/config/track.yaml'
DEFAULT_TRACK_META = REPO_ROOT / 'src/auto_drive_sim/models/road/track_meta.json'


def _load_csv(path: Path) -> list:
    with path.open('r', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def _to_world(drive: list, start_pose: dict) -> list:
    start_x = float(start_pose['x_m'])
    start_y = float(start_pose['y_m'])
    start_yaw = float(start_pose['yaw_rad'])
    cos_yaw = math.cos(start_yaw)
    sin_yaw = math.sin(start_yaw)
    world = []
    for point in drive:
        x_m = float(point['x_m'])
        y_m = float(point['y_m'])
        world.append({
            **point,
            'x_m': f'{start_x + x_m * cos_yaw - y_m * sin_yaw:.6f}',
            'y_m': f'{start_y + x_m * sin_yaw + y_m * cos_yaw:.6f}',
            'yaw_rad': f'{start_yaw + float(point["yaw_rad"]):.6f}',
        })
    return world


def _nearest_centerline(point: dict, centerline: list) -> dict:
    x_m = float(point['x_m'])
    y_m = float(point['y_m'])
    return min(
        centerline,
        key=lambda sample: (
            (x_m - float(sample['x_m'])) ** 2 +
            (y_m - float(sample['y_m'])) ** 2))


def _analyze(drive: list, centerline: list) -> dict:
    errors = []
    for point in drive:
        nearest = _nearest_centerline(point, centerline)
        yaw = float(nearest['yaw_rad'])
        dx = float(point['x_m']) - float(nearest['x_m'])
        dy = float(point['y_m']) - float(nearest['y_m'])
        errors.append(dx * -math.sin(yaw) + dy * math.cos(yaw))

    absolute = [abs(error) for error in errors]
    distance = 0.0
    for previous, current in zip(drive, drive[1:]):
        distance += math.hypot(
            float(current['x_m']) - float(previous['x_m']),
            float(current['y_m']) - float(previous['y_m']))

    first = drive[0]
    last = drive[-1]
    heading_change = 0.0
    previous_yaw = float(first['yaw_rad'])
    for point in drive[1:]:
        yaw = float(point['yaw_rad'])
        heading_change += math.atan2(
            math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
        previous_yaw = yaw

    absolute_sorted = sorted(absolute)
    index = min(len(absolute_sorted) - 1,
                int(round(0.95 * (len(absolute_sorted) - 1))))
    return {
        'samples': len(drive),
        'duration_s': round(float(last['t_s']) - float(first['t_s']), 3),
        'distance_m': round(distance, 3),
        'heading_change_rad': round(heading_change, 3),
        'max_abs_lateral_error_m': round(max(absolute), 3),
        'p95_abs_lateral_error_m': round(absolute_sorted[index], 3),
        'mean_abs_lateral_error_m': round(
            sum(absolute) / len(absolute), 3),
        'start': [round(float(first['x_m']), 3),
                  round(float(first['y_m']), 3),
                  round(float(first['yaw_rad']), 3)],
        'end': [round(float(last['x_m']), 3),
                round(float(last['y_m']), 3),
                round(float(last['yaw_rad']), 3)],
    }


def _plot(drive: list, centerline: list, lane_width: float,
          output: Path) -> None:
    center_x = [float(sample['x_m']) for sample in centerline]
    center_y = [float(sample['y_m']) for sample in centerline]
    left_x = []
    left_y = []
    right_x = []
    right_y = []
    for sample in centerline:
        yaw = float(sample['yaw_rad'])
        x_m = float(sample['x_m'])
        y_m = float(sample['y_m'])
        left_x.append(x_m - math.sin(yaw) * lane_width / 2.0)
        left_y.append(y_m + math.cos(yaw) * lane_width / 2.0)
        right_x.append(x_m + math.sin(yaw) * lane_width / 2.0)
        right_y.append(y_m - math.cos(yaw) * lane_width / 2.0)

    drive_x = [float(point['x_m']) for point in drive]
    drive_y = [float(point['y_m']) for point in drive]

    figure, axes = plt.subplots(figsize=(8, 8))
    axes.plot(center_x, center_y, '--', color='0.55', label='centerline')
    axes.plot(left_x, left_y, color='0.2', linewidth=1.0,
              label='lane edges')
    axes.plot(right_x, right_y, color='0.2', linewidth=1.0)
    axes.plot(drive_x, drive_y, color='tab:blue', linewidth=1.5,
              label='recorded drive')
    axes.plot(drive_x[0], drive_y[0], 'o', color='tab:green', label='start')
    axes.plot(drive_x[-1], drive_y[-1], 'o', color='tab:red', label='end')
    axes.set_aspect('equal', adjustable='box')
    axes.set_xlabel('x (m)')
    axes.set_ylabel('y (m)')
    axes.set_title('Recorded drive against the track centerline')
    axes.legend(loc='best')
    axes.grid(True, alpha=0.3)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=140, bbox_inches='tight')
    plt.close(figure)


def main() -> int:
    """Run the comparison and print a JSON summary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', type=Path, required=True)
    parser.add_argument('--centerline', type=Path,
                        default=DEFAULT_CENTERLINE)
    parser.add_argument('--track', type=Path, default=DEFAULT_TRACK)
    parser.add_argument('--track-meta', type=Path, default=DEFAULT_TRACK_META)
    parser.add_argument('--start-x', type=float)
    parser.add_argument('--start-y', type=float)
    parser.add_argument('--start-yaw', type=float)
    parser.add_argument('--output', type=Path,
                        default=REPO_ROOT / 'artifacts/drive_analysis.png')
    args = parser.parse_args()

    drive = _load_csv(args.drive)
    centerline = _load_csv(args.centerline)
    if len(drive) < 2:
        raise SystemExit('drive CSV needs at least two samples')
    if len(centerline) < 3:
        raise SystemExit('centerline CSV needs at least three samples')

    with args.track.open('r', encoding='utf-8') as stream:
        track = yaml.safe_load(stream)
    lane_width = float(track['track']['lane_width_m'])
    with args.track_meta.open('r', encoding='utf-8') as stream:
        start_pose = dict(json.load(stream)['start_pose'])
    if args.start_x is not None:
        start_pose['x_m'] = args.start_x
    if args.start_y is not None:
        start_pose['y_m'] = args.start_y
    if args.start_yaw is not None:
        start_pose['yaw_rad'] = args.start_yaw

    drive = _to_world(drive, start_pose)
    summary = _analyze(drive, centerline)
    summary['lane_width_m'] = lane_width
    summary['start_pose_world'] = [start_pose['x_m'], start_pose['y_m'],
                                   start_pose['yaw_rad']]
    summary['max_abs_lateral_error_ratio'] = round(
        summary['max_abs_lateral_error_m'] / (lane_width / 2.0), 3)
    _plot(drive, centerline, lane_width, args.output)
    summary['plot'] = str(args.output)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
