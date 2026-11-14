#!/usr/bin/env python3
"""Summarize scenario result JSON files into a Markdown table."""

import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = REPO_ROOT / 'results'
SCENARIOS = ['center', 'left', 'right']


def main() -> int:
    """Read result.json files and write results/summary.md."""
    rows = []
    for scenario in SCENARIOS:
        path = RESULTS_DIR / scenario / 'result.json'
        if not path.is_file():
            rows.append((scenario, None))
            continue
        rows.append((scenario, json.loads(path.read_text(encoding='utf-8'))))

    lines = [
        '# Scenario results',
        '',
        'Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, '
        'Gazebo Sim 8.15.0.',
        '',
        '| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | '
        'Boundary crossing | Perception valid (%) | Stop events |',
        '|---|---:|---:|---:|---:|---:|---:|---:|',
    ]
    all_laps = True
    all_clear = True
    all_p95 = True
    for scenario, result in rows:
        if result is None:
            lines.append(f'| {scenario} | missing | — | — | — | — | — | — |')
            all_laps = False
            all_clear = False
            all_p95 = False
            continue
        lap = 'yes' if result['lap_complete'] else 'no'
        if result['lap_time_s'] is None:
            lap_time = '—'
        else:
            lap_time = f"{result['lap_time_s']:.2f}"
        crossing = 'yes' if result['footprint_crossing'] else 'no'
        p95 = result['lateral_error_p95_abs_m']
        maximum = result['lateral_error_max_abs_m']
        valid = result['perception_valid_percent']
        stops = result['stop_events']
        lines.append(
            f'| {scenario} | {lap} | {lap_time} | {p95:.3f} | '
            f'{maximum:.3f} | {crossing} | {valid:.1f} | {stops} |')
        all_laps = all_laps and result['lap_complete']
        all_clear = all_clear and not result['footprint_crossing']
        all_p95 = all_p95 and p95 < 0.35

    passed = all_laps and all_clear and all_p95
    target = 'PASS' if passed else 'FAIL'
    lap_text = 'yes' if all_laps else 'no'
    clear_text = 'yes' if all_clear else 'no'
    p95_text = 'yes' if all_p95 else 'no'
    lines.extend([
        '',
        f'Release target: **{target}**',
        '',
        f'- All three laps complete: {lap_text}',
        f'- No footprint boundary crossing: {clear_text}',
        f'- p95 absolute lateral error below 0.35 m: {p95_text}',
        '',
    ])
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / 'summary.md').write_text(
        '\n'.join(lines), encoding='utf-8')
    print('\n'.join(lines))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
