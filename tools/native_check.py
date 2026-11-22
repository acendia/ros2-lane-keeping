#!/usr/bin/env python3
"""Validate native startup, Metal selection, timing, and baseline driving."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

from native_report import render_report
from native_runtime import (
    ROOT, Processes, RunLock, ValidationError, renderer_evidence,
    require_result, source_manifest, utc_now, write_json,
)


def run(command, **kwargs):
    """Compatibility helper used by process-failure regression tests."""
    return subprocess.run(command, check=True, cwd=ROOT, **kwargs)


def ready(processes, output, mode, timeout):
    acknowledgement = output / 'ready.json'
    processes.command('readiness', [sys.executable, 'tools/native_ready.py', mode,
        '--output', str(acknowledgement), '--timeout', str(timeout)], timeout + 5)
    value = require_result(acknowledgement, ('ready', 'clock_advances'))
    if value['ready'] is not True:
        raise ValidationError('ROS readiness did not pass')
    return value


def render_proof(processes, output, started_ns):
    log = Path.home() / '.gz/rendering/ogre2.log'
    evidence = renderer_evidence(log, started_ns, processes.owner_pids())
    shutil.copy2(log, output / 'ogre2.log')
    write_json(output / 'renderer.json', evidence)
    return evidence


def scoped_step(directory, action, report, name):
    directory.mkdir(parents=True)
    processes = Processes(directory)
    started = time.monotonic()
    step = {'name': name, 'status': 'running', 'started_utc': utc_now(),
            'artifacts': str(directory.relative_to(directory.parents[0]))}
    report['steps'].append(step)
    try:
        step.update(action(processes, directory))
        step['status'] = 'passed'
    except BaseException as error:
        step.update(status='interrupted' if isinstance(error, KeyboardInterrupt) else 'failed',
                    error=str(error) or type(error).__name__)
        raise
    finally:
        try:
            # A second terminal signal must not interrupt cleanup.
            previous = {sig: signal.signal(sig, signal.SIG_IGN)
                        for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                processes.close()
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
            step['cleanup'] = 'passed'
        except BaseException as error:
            step.update(cleanup='failed', cleanup_error=str(error), status='failed')
            raise
        finally:
            step['wall_duration_s'] = time.monotonic() - started
            write_json(directory / 'validation.json', step)


def camera_step(processes, output, args, timing=True):
    started_ns = time.time_ns()
    world = ROOT / 'tests/gazebo/camera_check.sdf'
    processes.start('gazebo', ['gz', 'sim', '-v', '4', '-s', '-r', str(world)])
    processes.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/camera_check/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    acknowledgement = ready(processes, output, 'camera', args.startup_timeout)
    evidence = render_proof(processes, output, started_ns)
    processes.command('image', [sys.executable, 'tools/camera_probe.py', '--duration', '2',
        '--output', str(output / 'frame.png')], 15)
    # camera_probe prints JSON to its log and exits nonzero on invalid pixels.
    image = json.loads((output / 'image.log').read_text())
    if image.get('valid') is not True or image.get('frames', 0) < 10:
        raise ValidationError('Image feasibility gate did not pass')
    image['output'] = 'frame.png'
    write_json(output / 'image.json', image)
    sdf = ET.parse(world).getroot()
    sensor = sdf.find('.//sensor[@type="camera"]')
    result = {'configured_camera': {name: sensor.findtext(name) for name in (
        'update_rate', 'camera/image/width', 'camera/image/height', 'camera/horizontal_fov')},
              'readiness': acknowledgement, 'renderer': evidence,
              'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest()}
    if timing:
        processes.command('timing', [sys.executable, 'tools/camera_timing_probe.py',
            '--duration', str(args.duration), '--output', str(output / 'timing.json')], args.duration + 15)
        measurement = require_result(output / 'timing.json', ('timing_pass', 'frames', 'max_sim_gap_s'))
        if measurement['timing_pass'] is not True or measurement['frames'] < 2:
            raise ValidationError('Camera timing gate did not pass')
        result['timing'] = measurement
    return result


def scenario_step(processes, output, args, scenario, offset):
    world = output / 'world.sdf'
    processes.command('world', [sys.executable, 'tools/make_scenario_world.py',
        '--offset', str(offset), '--output', str(world)], 10)
    started_ns = time.time_ns()
    processes.start('launch', ['ros2', 'launch', 'auto_drive_sim', 'demo.launch.py',
        'gui:=false', 'controller:=false', f'world:={world}'])
    processes.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'bridge_node', '--ros-args',
        '-p', f'config_file:={ROOT / "src/auto_drive_sim/config/eval_bridge.yaml"}'])
    acknowledgement = ready(processes, output, 'lane', args.startup_timeout)
    evidence = render_proof(processes, output, started_ns)
    evaluator_ready = output / 'evaluator-ready.json'
    evaluator = processes.start('evaluator', ['ros2', 'run', 'auto_drive_sim', 'evaluator',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', f'scenario:={scenario}',
        '-p', f'start_y:={15.0 + offset}', '-p', f'output_dir:={output}',
        '-p', f'ready_file:={evaluator_ready}', '-p', 'timeout_s:=130.0'])
    evaluator_ack = processes.wait_file(evaluator_ready, args.startup_timeout)
    if evaluator_ack.get('ready') is not True:
        raise ValidationError('Evaluator has not processed a pose')
    processes.start('controller', ['ros2', 'run', 'auto_drive_sim', 'lane_controller',
        '--ros-args', '-p', 'use_sim_time:=true'])
    started = time.monotonic()
    processes.wait_for(evaluator, args.scenario_timeout)
    result = require_result(output / 'result.json', (
        'lap_complete', 'footprint_crossing', 'lateral_error_p95_abs_m', 'duration_s', 'samples'))
    if (result['lap_complete'] is not True or result['footprint_crossing'] is not False or
            not 0 <= result['lateral_error_p95_abs_m'] < 0.35 or result['samples'] < 2):
        raise ValidationError(f'{scenario} did not meet the baseline lap target')
    return {'readiness': acknowledgement, 'evaluator_ready': evaluator_ack, 'renderer': evidence,
            'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
            'evaluation_wall_s': time.monotonic() - started, 'simulation_duration_s': result['duration_s'],
            'result': result}


def execute(args, output):
    report = {'schema_version': 1, 'mode': args.mode, 'status': 'running',
              'started_utc': utc_now(), 'steps': [], 'settings': vars(args)}
    started = time.monotonic()
    write_json(output / 'validation.json', report)
    stage = 'lock'
    try:
        with RunLock(args.domain):
            os.environ['GZ_PARTITION'] = f'native_check_{output.name}'
            os.environ['ROS_DOMAIN_ID'] = str(args.domain)
            stage = 'preflight'

            def environment(processes, directory):
                command = [sys.executable, 'tools/native_runtime.py', '--output', str(directory / 'environment.json')]
                if args.mode == 'scenarios':
                    command.append('--project')
                processes.command('preflight', command, 60)
                return require_result(directory / 'environment.json', ('architecture', 'ros_bundle_version', 'gazebo'))

            scoped_step(output / 'preflight', environment, report, 'preflight')
            manifest = source_manifest()
            metadata = {
                'started_utc': report['started_utc'], 'commit': subprocess.check_output(
                    ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
                'source_files': manifest, 'settings': vars(args),
                'gz_partition': os.environ['GZ_PARTITION'], 'ros_domain_id': args.domain}
            write_json(output / 'metadata.json', metadata)
            if args.require_clean and metadata['dirty']:
                raise ValidationError('Clean source required; commit changes before publishing this run')
            stage = args.mode
            if args.mode in ('camera', 'startup'):
                count = args.repetitions if args.mode == 'startup' else 1
                for index in range(count):
                    scoped_step(output / f'camera-{index + 1:02}',
                        lambda p, d: camera_step(p, d, args, args.mode == 'camera'), report, f'camera-{index + 1:02}')
            elif args.mode == 'scenarios':
                for scenario, offset in [('center', 0.0), ('left', 0.3), ('right', -0.3)]:
                    scoped_step(output / scenario,
                        lambda p, d, s=scenario, o=offset: scenario_step(p, d, args, s, o), report, scenario)
                summary = Processes(output)
                try:
                    summary.command('summary', [sys.executable, 'tools/summarize_results.py',
                        '--results-dir', str(output), '--environment', 'Native macOS arm64, verified hardware Metal.'], 15)
                finally:
                    summary.close()
            if source_manifest() != manifest:
                raise ValidationError('Source files changed during validation; rerun a stable revision')
            report['status'] = 'passed'
    except KeyboardInterrupt:
        report.update(status='interrupted', stage=stage, error='Interrupted; owned processes were stopped')
    except Exception as error:
        report.update(status='failed', stage=stage, error=str(error))
    finally:
        report.update(finished_utc=utc_now(), wall_duration_s=time.monotonic() - started)
        write_json(output / 'validation.json', report)
        metadata_path = output / 'metadata.json'
        metadata = json.loads(metadata_path.read_text()) if metadata_path.is_file() else None
        (output / 'summary.md').write_text(render_report(report, metadata))
        print(f'{report["status"]}: {output}', flush=True)
        if 'error' in report:
            print(report['error'], file=sys.stderr)
    return 0 if report['status'] == 'passed' else 130 if report['status'] == 'interrupted' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['preflight', 'startup', 'camera', 'scenarios'])
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--repetitions', type=int, default=10)
    parser.add_argument('--domain', type=int, default=int(os.environ.get('NATIVE_ROS_DOMAIN_ID', '72')))
    parser.add_argument('--startup-timeout', type=float, default=45)
    parser.add_argument('--scenario-timeout', type=float, default=420)
    parser.add_argument('--require-clean', action='store_true')
    args = parser.parse_args()
    if min(args.duration, args.repetitions, args.startup_timeout, args.scenario_timeout) <= 0:
        parser.error('Durations, timeouts, and repetitions must be positive')
    base = ROOT / 'artifacts/native-validation'
    base.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=args.mode + '-', dir=base))
    print(f'Output: {output}', flush=True)

    def interrupt(*_):
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupt)
    return execute(args, output)


if __name__ == '__main__':
    raise SystemExit(main())
