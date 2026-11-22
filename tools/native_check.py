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
import yaml

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
    start_y = 15.0
    yaw = 0.0
    model_path = None
    track_hashes = {}
    evaluator_track = []
    track = ROOT / 'src/auto_drive_sim/config/track_tight_worn.yaml'
    if args.tight_worn:
        model_path = output / 'models'
        road = model_path / 'road_tight_worn'
        processes.command('track', [sys.executable, 'tools/generate_track.py',
            '--config', str(track), '--output', str(road)], 15)
        generated = require_result(road / 'track_meta.json', ('start_pose',))
        track_hashes = {
            str(path.relative_to(road)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(road.rglob('*')) if path.is_file()
        }
        start_y = generated['start_pose']['y_m']
        yaw = -0.08 if offset > 0 else 0.08 if offset < 0 else 0.0
        evaluator_track = ['-p', f'centerline_path:={road / "centerline.csv"}',
                           '-p', f'track_config_path:={track}']
    processes.command('world', [sys.executable, 'tools/make_scenario_world.py',
        '--offset', str(offset), '--output', str(world), '--source-world',
        str(ROOT / 'src/auto_drive_sim/worlds' / args.world),
        '--start-y', str(start_y)], 10)
    launch = ['ros2', 'launch', 'auto_drive_sim', 'demo.launch.py',
              'gui:=false', 'controller:=false', f'world:={world}']
    launch_env = None
    if model_path is not None:
        sdf = ET.parse(world)
        includes = {include.findtext('name'): include for include in sdf.findall('./world/include')}
        includes['road'].find('uri').text = 'model://road_tight_worn'
        includes['car'].find('pose').text = f'-10 {start_y + offset} 0.05 0 0 {yaw}'
        sdf.write(world, encoding='utf-8')
        existing = os.environ.get('GZ_SIM_RESOURCE_PATH')
        resources = os.pathsep.join([str(model_path), existing] if existing else [str(model_path)])
        launch_env = dict(os.environ, GZ_SIM_RESOURCE_PATH=resources)
    started_ns = time.time_ns()
    processes.start('launch', launch, env=launch_env)
    processes.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'bridge_node', '--ros-args',
        '-p', f'config_file:={ROOT / "src/auto_drive_sim/config/eval_bridge.yaml"}'])
    acknowledgement = ready(processes, output, 'lane', args.startup_timeout)
    evidence = render_proof(processes, output, started_ns)
    evaluator_ready = output / 'evaluator-ready.json'
    evaluator = processes.start('evaluator', ['ros2', 'run', 'auto_drive_sim', 'evaluator',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', f'scenario:={scenario}',
        '-p', f'start_y:={start_y + offset}', '-p', f'lane_center_y:={start_y}',
        '-p', f'start_yaw:={yaw}', '-p', f'output_dir:={output}',
        '-p', f'ready_file:={evaluator_ready}', '-p', 'timeout_s:=130.0', *evaluator_track])
    evaluator_ack = processes.wait_file(evaluator_ready, args.startup_timeout)
    if evaluator_ack.get('ready') is not True:
        raise ValidationError('Evaluator has not processed a pose')
    processes.start('controller', ['ros2', 'run', 'auto_drive_sim', 'lane_controller',
        '--ros-args', '-p', 'use_sim_time:=true'])
    started = time.monotonic()
    processes.wait_for(evaluator, args.scenario_timeout)
    result = require_result(output / 'result.json', (
        'lap_complete', 'footprint_crossing', 'lateral_error_p95_abs_m', 'duration_s', 'samples'))
    return {'readiness': acknowledgement, 'evaluator_ready': evaluator_ack, 'renderer': evidence,
            'generated_track_sha256': track_hashes,
            'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
            'evaluation_wall_s': time.monotonic() - started, 'simulation_duration_s': result['duration_s'],
            'result': result}


def lap_passed(result):
    return (result['lap_complete'] is True and result['footprint_crossing'] is False and
            0 <= result['lateral_error_p95_abs_m'] < 0.35 and result['samples'] >= 2)


def make_straight_track(processes, output):
    config = yaml.safe_load((ROOT / 'src/auto_drive_sim/config/track_two_lane.yaml').read_text())
    config['track']['straight_length_m'] = 90.0
    config['model_name'] = 'road_scenario_straight'
    config_path = output / 'track.yaml'
    config_path.write_text(yaml.safe_dump(config))
    road = output / 'models' / config['model_name']
    processes.command('track', [sys.executable, 'tools/generate_track.py',
        '--config', str(config_path), '--output', str(road)], 15)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH')
    paths = [str(road.parent)] + ([existing] if existing else [])
    return config, config_path, road, dict(os.environ, GZ_SIM_RESOURCE_PATH=os.pathsep.join(paths))


def straight_scenario_world(source, output, road, ego_y, *, lead=False):
    sdf = ET.parse(source)
    includes = {item.findtext('name'): item for item in sdf.findall('./world/include')}
    includes['road'].find('uri').text = f'model://{road.name}'
    includes['car'].find('pose').text = f'-35 {ego_y} 0.05 0 0 0'
    if lead:
        includes['lead_vehicle'].find('pose').text = f'-20 {ego_y} 0.05 0 0 0'
    world = output / 'world.sdf'
    sdf.write(world, encoding='utf-8')
    return world


def lead_step(processes, output, args, scenario):
    moving = scenario in ('moving', 'braking')
    suffix = '_moving_lead' if moving else '_lead'
    world = ROOT / 'src/auto_drive_sim/worlds' / f'car_world_two_lane{suffix}.sdf'
    launch_env = None
    probe_track = []
    if scenario == 'braking':
        config, _, road, launch_env = make_straight_track(processes, output)
        track = config['track']
        ego_y = track['curve_radius_m'] - (track['lane_width_m'] + track['marking_width_m']) / 2
        world = straight_scenario_world(world, output, road, ego_y, lead=True)
        probe_track = ['--centerline', str(road / 'centerline.csv')]
    started_ns = time.time_ns()
    processes.start('launch', ['ros2', 'launch', 'auto_drive_sim', 'demo.launch.py',
        'gui:=false', 'controller:=false', 'lane_strategy:=rightmost',
        f'lead_driver:={str(moving).lower()}',
        f'lead_braking_scenario:={str(scenario == "braking").lower()}', f'world:={world}'], env=launch_env)
    processes.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'bridge_node', '--ros-args',
        '-p', f'config_file:={ROOT / "src/auto_drive_sim/config/eval_bridge.yaml"}'])
    if moving:
        processes.start('lead-pose-bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/lead_vehicle/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '--ros-args', '-r', '/model/lead_vehicle/pose:=/evaluation/lead_pose'])
    acknowledgement = ready(processes, output, 'lane', args.startup_timeout)
    evidence = render_proof(processes, output, started_ns)
    detector = processes.start('detector', [sys.executable, '-m', 'auto_drive_sim.lead_detector',
        '--ros-args', '-p', 'use_sim_time:=true'], required=scenario != 'loss')
    probe = processes.start('probe', [sys.executable, 'tools/lead_scenario_probe.py', scenario,
        str(output), str(world), *probe_track, '--ros-args', '-p', 'use_sim_time:=true'])
    processes.wait_file(output / 'probe-ready.json', args.startup_timeout)
    processes.start('controller', ['ros2', 'run', 'auto_drive_sim', 'lane_controller',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'lead_enabled:=true'])
    if scenario == 'loss':
        processes.wait_file(output / 'cut-detector.json', args.startup_timeout)
        if detector.poll() is not None:
            raise ValidationError('Detector stopped before the requested fault')
        detector.send_signal(signal.SIGINT)
        processes.wait_for(detector, 5)
    processes.wait_for(probe, args.scenario_timeout)
    result = require_result(output / 'result.json', ('pass', 'pose_samples'))
    return {'readiness': acknowledgement, 'renderer': evidence,
            'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(), 'result': result}


def lane_step(processes, output, args, scenario):
    source = 'rightmost' if scenario == 'change-left' else 'leftmost'
    target = 'rightmost' if scenario == 'change-right' else 'leftmost'
    config_dir = ROOT / 'src/auto_drive_sim/config'
    config, config_path, road, launch_env = make_straight_track(processes, output)

    center = config['track']['curve_radius_m']
    width = config['track']['lane_width_m'] + config['track']['marking_width_m']
    start_y = center + (0.5 if source == 'leftmost' else -0.5) * width
    world = straight_scenario_world(ROOT / 'src/auto_drive_sim/worlds/car_world_two_lane.sdf',
                                    output, road, start_y)
    started_ns = time.time_ns()
    launch = ['ros2', 'launch', 'auto_drive_sim', 'demo.launch.py',
              'gui:=false', 'controller:=false', f'lane_strategy:={source}', f'world:={world}']
    if source != target:
        launch.append(f'lane_change_to:={target}')
    processes.start('launch', launch, env=launch_env)
    processes.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'bridge_node', '--ros-args',
        '-p', f'config_file:={config_dir / "eval_bridge.yaml"}'])
    acknowledgement = ready(processes, output, 'lane', args.startup_timeout)
    evidence = render_proof(processes, output, started_ns)
    probe = processes.start('probe', [sys.executable, 'tools/lane_scenario_probe.py',
        str(output), str(center), str(width), source, target, '--ros-args', '-p', 'use_sim_time:=true'])
    processes.wait_file(output / 'probe-ready.json', args.startup_timeout)
    processes.start('controller', ['ros2', 'run', 'auto_drive_sim', 'lane_controller',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'target_speed_mps:=1.2'])
    processes.wait_for(probe, args.scenario_timeout)
    result = require_result(output / 'result.json', ('pass', 'divider_crossings'))
    return {'readiness': acknowledgement, 'renderer': evidence,
            'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
            'track_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
            'result': result}


def intersection_step(processes, output, args, scenario):
    config, config_path, road, launch_env = make_straight_track(processes, output)
    track = config['track']
    half_width = (track['lane_width_m'] + track['marking_width_m']) / 2
    lane_y = track['curve_radius_m'] - half_width
    world = straight_scenario_world(ROOT / 'src/auto_drive_sim/worlds/car_world_two_lane.sdf',
                                    output, road, lane_y)
    sdf = ET.parse(world)
    scene = sdf.find('world')
    line_center_x, line_depth = -24.0, 0.34
    goal_x, search_x = -7.0, -29.0
    traffic = scenario == 'cross-traffic'
    scene.append(ET.fromstring('''
        <model name="cross_street"><static>true</static>
          <pose>-18 15 0.001 0 0 0</pose><link name="road"><visual name="asphalt">
            <geometry><box><size>7.2 36 0.005</size></box></geometry>
            <material><ambient>0.20 0.20 0.22 1</ambient>
              <diffuse>0.22 0.22 0.24 1</diffuse></material>
          </visual></link>
        </model>'''))
    def paint(name, x, y, dx, dy, color, height=0.018, thickness=0.004):
        scene.append(ET.fromstring(f'''
            <model name="{name}"><static>true</static><pose>{x} {y} {height} 0 0 0</pose>
              <link name="surface"><visual name="surface">
                <geometry><box><size>{dx} {dy} {thickness}</size></box></geometry>
                <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material>
              </visual></link>
            </model>'''))

    for index, y in enumerate((1, 5, 9, 21, 25, 29)):
        paint(f'cross_center_{index}', -18, y, 0.11, 2.4, '0.88 0.72 0.10')
    for side, x in enumerate((-22.2, -13.8)):
        for end, y in enumerate((5, 25)):
            paint(f'sidewalk_{side}_{end}', x, y, 1.2, 11.4, '0.53 0.53 0.51', 0.06, 0.12)
        for index in range(12):
            paint(f'crosswalk_{side}_{index}', x, 11.65 + 0.60 * index,
                  1.2, 0.24, '0.68 0.52 0.38')
    if traffic:
        scene.append(ET.fromstring('''
            <include><uri>model://lead_car</uri><name>lead_vehicle</name>
              <pose>-16.25 8.5 0.05 0 0 1.57079632679</pose>
            </include>'''))
    if scenario != 'missing-line':
        scene.append(ET.fromstring(f'''
            <model name="stop_line"><static>true</static>
              <pose>{line_center_x} {lane_y} 0.018 0 0 0</pose><link name="paint"><visual name="stripe">
                <geometry><box><size>{line_depth} 2.60 0.004</size></box></geometry>
                <material><ambient>0.95 0.95 0.95 1</ambient>
                  <diffuse>0.95 0.95 0.95 1</diffuse></material>
              </visual></link>
            </model>'''))
    sdf.write(world, encoding='utf-8')
    started_ns = time.time_ns()
    processes.start('launch', ['ros2', 'launch', 'auto_drive_sim', 'demo.launch.py',
        'gui:=false', 'controller:=false', 'lane_strategy:=rightmost', f'world:={world}'],
        env=launch_env)
    processes.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'bridge_node', '--ros-args',
        '-p', f'config_file:={ROOT / "src/auto_drive_sim/config/eval_bridge.yaml"}'])
    if traffic:
        processes.start('cross-bridge', ['ros2', 'run', 'ros_gz_bridge', 'bridge_node', '--ros-args',
            '-p', f'config_file:={ROOT / "src/auto_drive_sim/config/lead_bridge.yaml"}'])
        processes.start('cross-pose-bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/lead_vehicle/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '--ros-args', '-r', '/model/lead_vehicle/pose:=/evaluation/lead_pose'])
        processes.start('cross-traffic', [sys.executable, 'tools/cross_traffic.py',
            '--ros-args', '-p', 'use_sim_time:=true'])
    acknowledgement = ready(processes, output, 'lane', args.startup_timeout)
    evidence = render_proof(processes, output, started_ns)
    command = [sys.executable, 'tools/intersection_probe.py', str(output),
               '--lane-y', str(lane_y), '--lane-half-width', str(half_width),
               '--line-x', str(line_center_x - line_depth / 2),
               '--goal-x', str(goal_x), '--search-x', str(search_x)]
    if scenario == 'missing-line':
        command.append('--missing-line')
    if traffic:
        command.append('--cross-traffic')
    probe = processes.start('probe', [*command, '--ros-args', '-p', 'use_sim_time:=true'])
    processes.wait_file(output / 'probe-ready.json', args.startup_timeout)
    processes.start('intersection-controller', ['ros2', 'run', 'auto_drive_sim', 'intersection_controller',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', f'goal_m:={goal_x + 35.0}',
        '-p', f'search_limit_m:={search_x + 35.0}',
        '-p', f'yield_to_traffic:={str(traffic).lower()}'])
    processes.start('controller', ['ros2', 'run', 'auto_drive_sim', 'lane_controller',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'target_speed_mps:=1.2',
        '-r', '/car/cmd_vel:=/car/lane_cmd_vel'])
    processes.wait_for(probe, args.scenario_timeout)
    result = require_result(output / 'result.json', ('pass', 'final_phase'))
    return {'readiness': acknowledgement, 'renderer': evidence,
            'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
            'track_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
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
                if args.mode in ('scenarios', 'lead', 'lanes', 'intersection'):
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
                failed = []
                for scenario, offset in [('center', 0.0), ('left', 0.3), ('right', -0.3)]:
                    scoped_step(output / scenario,
                        lambda p, d, s=scenario, o=offset: scenario_step(p, d, args, s, o), report, scenario)
                    step = report['steps'][-1]
                    if not lap_passed(step['result']):
                        step.update(status='failed', error='Lap acceptance criteria were not met')
                        write_json(output / scenario / 'validation.json', step)
                        failed.append(scenario)
                if failed:
                    raise ValidationError(f'Scenarios did not meet lap criteria: {", ".join(failed)}')
            elif args.mode in ('lead', 'lanes', 'intersection'):
                failed = []
                if args.mode == 'lead':
                    scenarios = args.lead_case or ('static', 'moving', 'braking', 'loss')
                    step_action = lead_step
                elif args.mode == 'lanes':
                    scenarios = ('select-left', 'change-left', 'change-right')
                    step_action = lane_step
                else:
                    scenarios = ('stop-and-cross', 'missing-line', 'cross-traffic')
                    step_action = intersection_step
                for scenario in scenarios:
                    scoped_step(output / scenario,
                        lambda p, d, s=scenario: step_action(p, d, args, s), report, scenario)
                    step = report['steps'][-1]
                    if step['result']['pass'] is not True:
                        step.update(status='failed', error='Scenario criteria were not met')
                        write_json(output / scenario / 'validation.json', step)
                        failed.append(scenario)
                if failed:
                    raise ValidationError(f'Scenarios did not meet criteria: {", ".join(failed)}')
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
    parser.add_argument('mode', choices=['preflight', 'startup', 'camera', 'scenarios', 'lead', 'lanes',
                                         'intersection'])
    parser.add_argument('--lead-case', choices=('static', 'moving', 'braking', 'loss'), nargs='+',
                        help='Run selected lead-vehicle cases')
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--repetitions', type=int, default=10)
    parser.add_argument('--domain', type=int, default=int(os.environ.get('NATIVE_ROS_DOMAIN_ID', '72')))
    parser.add_argument('--startup-timeout', type=float, default=45)
    parser.add_argument('--scenario-timeout', type=float, default=420)
    parser.add_argument('--world', choices=['car_world.sdf', 'car_world_lighting.sdf'],
                        default='car_world.sdf', help='Scenario world for lane-keeping evaluation')
    parser.add_argument('--tight-worn', action='store_true',
                        help='Generate a tighter oval with a narrower lane and worn markings')
    parser.add_argument('--require-clean', action='store_true')
    args = parser.parse_args()
    if args.tight_worn and args.mode != 'scenarios':
        parser.error('--tight-worn requires scenarios mode')
    if args.lead_case and (args.mode != 'lead' or len(set(args.lead_case)) != len(args.lead_case)):
        parser.error('--lead-case requires lead mode and distinct cases')
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
