#!/usr/bin/env python3
"""Generate a readable report and export clean, successful native evidence."""
import argparse
import json
from pathlib import Path
import re
import shutil


def render_report(report, metadata=None):
    metadata = metadata or {}
    commit = metadata.get('commit', 'unavailable')
    lines = ['# Native validation', '', f'Mode: `{report["mode"]}`', '',
             f'Status: **{report["status"].upper()}**', '',
             f'Source revision: `{commit}`; dirty at run start: `{metadata.get("dirty", "unknown")}`.',
             '', f'Started (UTC): {report["started_utc"]}', '',
             f'Wall duration: {report.get("wall_duration_s", 0):.2f} s.', '',
             '[Raw validation](validation.json) · [Source provenance](metadata.json)', '']
    if report.get('error'):
        lines += [f'Failure stage: `{report.get("stage", "unknown")}`', '', report['error'], '']
    if report['mode'] in ('camera', 'startup'):
        lines += ['| Check | Status | Device | Max sim gap (s) | p95 frame age (s) | Cleanup |',
                  '|---|---|---|---:|---:|---|']
        for step in report['steps']:
            if step['name'] == 'preflight':
                continue
            timing = step.get('timing', {})
            gap = f'{timing["max_sim_gap_s"]:.4f}' if 'max_sim_gap_s' in timing else 'not sampled'
            age = f'{timing["arrival_age_p95_s"]:.4f}' if 'arrival_age_p95_s' in timing else 'not sampled'
            lines.append(f'| [{step["name"]}]({step["name"]}/validation.json) | {step["status"]} | '
                         f'{step.get("renderer", {}).get("device", "unverified")} | {gap} | {age} | {step.get("cleanup", "unknown")} |')
        if report['mode'] == 'startup':
            lines += ['', 'Startup checks establish fresh images and readiness; they do not replace a sustained camera timing test.']
    if report['mode'] == 'scenarios':
        lines += ['| Scenario | Status | Lap time (sim s) | p95 error (m) | Crossing | Stops | Cleanup |',
                  '|---|---|---:|---:|---|---:|---|']
        for step in report['steps']:
            if step['name'] == 'preflight':
                continue
            result = step.get('result', {})
            lap = result.get('lap_time_s')
            error = result.get('lateral_error_p95_abs_m')
            lines.append(f'| [{step["name"]}]({step["name"]}/result.json) | {step["status"]} | '
                         f'{lap if lap is not None else "missing"} | {error if error is not None else "missing"} | '
                         f'{result.get("footprint_crossing", "missing")} | {result.get("stop_events", "missing")} | {step.get("cleanup", "unknown")} |')
    if report['mode'] == 'lead':
        lines += ['| Scenario | Status | Duration (sim s) | Min footprint gap bound (m) | Final state |',
                  '|---|---|---:|---:|---|']
        for step in report['steps']:
            if step['name'] == 'preflight':
                continue
            result = step.get('result', {})
            duration = result.get('duration_s')
            duration_text = f'{duration:.2f}' if duration is not None else 'missing'
            gap = result.get('minimum_footprint_separation_bound_m')
            gap_text = f'{gap:.3f}' if gap is not None else 'missing'
            lines.append(f'| [{step["name"]}]({step["name"]}/result.json) | {step["status"]} | '
                         f'{duration_text} | {gap_text} | {result.get("final_state", "missing")} |')
    return '\n'.join(lines) + '\n'


def public_json(value):
    """Remove the local home path from publishable records, retaining structure."""
    if isinstance(value, str):
        return value.replace(str(Path.home()), '~')
    if isinstance(value, dict):
        return {key: public_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [public_json(item) for item in value]
    return value


def export_run(source, output):
    source, output = Path(source), Path(output)
    report = json.loads((source / 'validation.json').read_text())
    metadata = json.loads((source / 'metadata.json').read_text())
    if report.get('status') != 'passed' or metadata.get('dirty') is not False:
        raise ValueError('Only successful runs from clean source revisions can be published')
    if not re.fullmatch('[0-9a-f]{40}', metadata.get('commit', '')):
        raise ValueError('Missing source commit')
    if not report.get('steps'):
        raise ValueError('Missing validation steps')
    for step in report['steps']:
        name = step['name']
        if not re.fullmatch(r'[a-z0-9-]+', name):
            raise ValueError('Invalid step name')
        directory = source / name
        required = ['validation.json', 'environment.json'] if name == 'preflight' else [
            'validation.json', 'ready.json', 'renderer.json']
        if name != 'preflight':
            if report['mode'] == 'scenarios':
                required += ['result.json', 'lateral_error.csv', 'evaluator-ready.json']
            elif report['mode'] == 'lead':
                required += ['result.json', 'trajectory.csv', 'telemetry.json', 'probe-ready.json']
                if name == 'loss':
                    required.append('cut-detector.json')
            else:
                required += ['image.json', 'frame.png']
            if report['mode'] == 'camera':
                required.append('timing.json')
        if any(not (directory / file).is_file() for file in required):
            raise ValueError(f'Missing raw evidence for {name}')
        if json.loads((directory / 'validation.json').read_text()) != step:
            raise ValueError(f'Step record does not match run record: {name}')
        if step.get('status') != 'passed' or step.get('cleanup') != 'passed':
            raise ValueError(f'Unqualified step: {name}')
    # Do not overwrite or combine evidence from different invocations.
    output.mkdir(parents=True, exist_ok=False)
    for name, value in [('validation.json', report), ('metadata.json', metadata)]:
        (output / name).write_text(json.dumps(public_json(value), indent=2) + '\n')
    for step in report['steps']:
        name = step['name']
        if not re.fullmatch(r'[a-z0-9-]+', name):
            raise ValueError('Invalid step name')
        directory = output / name
        directory.mkdir()
        for file in ('validation.json', 'environment.json', 'ready.json', 'evaluator-ready.json',
                     'renderer.json', 'timing.json', 'image.json', 'result.json',
                     'telemetry.json', 'probe-ready.json', 'cut-detector.json'):
            path = source / name / file
            if path.is_file():
                value = json.loads(path.read_text())
                (directory / file).write_text(json.dumps(public_json(value), indent=2) + '\n')
        for file in ('frame.png', 'lateral_error.csv', 'trajectory.csv'):
            path = source / name / file
            if path.is_file():
                shutil.copy2(path, directory / file)
        log = source / name / 'ogre2.log'
        if log.is_file():
            lines = [line for line in log.read_text().splitlines() if 'Metal: Requested' in line]
            (directory / 'renderer.txt').write_text('\n'.join(lines) + '\n')
    (output / 'summary.md').write_text(render_report(public_json(report), public_json(metadata)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    export_run(args.source, args.output)
    print(args.output / 'summary.md')


if __name__ == '__main__':
    main()
