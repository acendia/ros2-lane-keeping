#!/usr/bin/env python3
"""Run isolated camera or baseline checks in a sourced native ROS environment."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    return subprocess.run(command, check=True, cwd=ROOT, **kwargs)


class Processes:
    def __init__(self, output):
        self.output = output
        self.children = []

    def start(self, name, command):
        stream = (self.output / f'{name}.log').open('w')
        try:
            child = subprocess.Popen(command, cwd=ROOT, stdout=stream,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        finally:
            stream.close()
        self.children.append(child)
        return child

    def close(self):
        for child in reversed(self.children):
            if child.poll() is not None:
                continue
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(1)
        for child in self.children:
            if child.poll() is not None:
                continue
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['camera', 'scenarios'])
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()
    base = ROOT / 'artifacts/native-validation'
    base.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=args.mode + '-', dir=base))
    os.environ['GZ_PARTITION'] = f'native_check_{os.getpid()}'
    os.environ['ROS_DOMAIN_ID'] = '72'
    metadata = {
        'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
        'platform': platform.platform(), 'python': sys.version,
        'source_sha256': hashlib.sha256(b''.join(
            p.read_bytes() for folder in ('src', 'tools')
            for p in sorted((ROOT / folder).rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts)).hexdigest(),
    }
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'Output: {output}', flush=True)
    if args.mode == 'camera':
        processes = Processes(output)
        try:
            processes.start('gazebo', ['gz', 'sim', '-v', '4', '-s', '-r',
                                      str(ROOT / 'tests/gazebo/camera_check.sdf')])
            processes.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/camera_check/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
            run(['ros2', 'topic', 'echo', '/camera_check/image', 'sensor_msgs/msg/Image', '--once'],
                stdout=subprocess.DEVNULL, timeout=180)
            run([sys.executable, 'tools/camera_probe.py', '--duration', '5',
                 '--output', str(output / 'frame.png')], stdout=(output / 'image.json').open('w'))
            run([sys.executable, 'tools/camera_timing_probe.py', '--duration', str(args.duration),
                 '--output', str(output / 'timing.json')])
        finally:
            renderer_log = Path.home() / '.gz/rendering/ogre2.log'
            if renderer_log.is_file():
                shutil.copy2(renderer_log, output / 'ogre2.log')
            processes.close()
    else:
        for scenario, offset in [('center', 0.0), ('left', 0.3), ('right', -0.3)]:
            directory = output / scenario
            directory.mkdir()
            processes = Processes(directory)
            try:
                run([sys.executable, 'tools/make_scenario_world.py', '--offset', str(offset),
                     '--output', str(directory / 'world.sdf')])
                processes.start('launch', ['ros2', 'launch', 'auto_drive_sim', 'demo.launch.py',
                    'gui:=false', 'controller:=false', f'world:={directory / "world.sdf"}'])
                processes.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'bridge_node',
                    '--ros-args', '-p', f'config_file:={ROOT / "src/auto_drive_sim/config/eval_bridge.yaml"}'])
                run(['ros2', 'topic', 'echo', '/car/lane_estimate', 'auto_drive_sim_interfaces/msg/LaneEstimate', '--once'],
                    stdout=subprocess.DEVNULL, timeout=180)
                evaluator = processes.start('evaluator', ['ros2', 'run', 'auto_drive_sim', 'evaluator',
                    '--ros-args', '-p', 'use_sim_time:=true', '-p', f'scenario:={scenario}',
                    '-p', f'start_y:={15.0 + offset}', '-p', f'output_dir:={directory}',
                    '-p', 'timeout_s:=130.0'])
                time.sleep(2)
                processes.start('controller', ['ros2', 'run', 'auto_drive_sim', 'lane_controller',
                    '--ros-args', '-p', 'use_sim_time:=true'])
                rc = evaluator.wait(timeout=420)
                if rc != 0 or not (directory / 'result.json').is_file():
                    raise RuntimeError(f'{scenario}: evaluator failed (exit {rc})')
            finally:
                processes.close()
        run([sys.executable, 'tools/summarize_results.py', '--results-dir', str(output),
             '--environment', 'Native macOS arm64, community Jazzy/Harmonic, Metal experiment.'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
