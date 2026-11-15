#!/usr/bin/env python3
"""Create a scenario world with the car offset from the centerline."""

import argparse
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_WORLD = REPO_ROOT / 'src/auto_drive_sim/worlds/car_world.sdf'
DEFAULT_OUTPUT = REPO_ROOT / 'artifacts/scenarios/scenario_world.sdf'


def main() -> int:
    """Write the scenario world."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offset', type=float, default=0.0,
                        help='positive offset is left of the centerline')
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--source-world', type=Path, default=SOURCE_WORLD)
    args = parser.parse_args()

    text = args.source_world.read_text(encoding='utf-8')
    old_pose = '<pose>-10 15 0.05 0 0 0</pose>'
    if text.count(old_pose) != 1:
        raise SystemExit('could not find the car start pose')
    new_pose = f'<pose>-10 {15.0 + args.offset:.6f} 0.05 0 0 0</pose>'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        text.replace(old_pose, new_pose), encoding='utf-8', newline='\n')
    print(args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
