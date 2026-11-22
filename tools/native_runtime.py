"""Process supervision and inspectable evidence for native validation."""
import contextlib
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import psutil

ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class ValidationError(RuntimeError):
    pass


class RunLock:
    """Serialize checks sharing Ogre's global log, and reserve the ROS domain."""
    def __init__(self, domain):
        if not 0 <= domain <= 100:
            raise ValidationError('ROS domain must be in 0..100 for this local profile')
        self.domain = domain
        self.streams = []

    def __enter__(self):
        try:
            for name in ('renderer', f'domain-{self.domain}'):
                path = Path(tempfile.gettempdir()) / f'auto-drive-native-{os.getuid()}-{name}.lock'
                stream = path.open('a+')
                self.streams.append(stream)
                try:
                    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as error:
                    raise ValidationError(f'Another native check owns {name}; wait for it to finish') from error
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_):
        for stream in self.streams:
            stream.close()
        self.streams.clear()


class Processes:
    """Own process groups, monitor failures, and reap leaders and descendants."""
    def __init__(self, output):
        self.output = Path(output)
        self.children = []
        self.records = []
        self.descendants = {}
        self.closed = False

    def start(self, name, command, required=True, *, env=None):
        with (self.output / f'{name}.log').open('w') as stream:
            child = subprocess.Popen(command, cwd=ROOT, stdout=stream,
                                     stderr=subprocess.STDOUT, start_new_session=True,
                                     env=env)
        self.children.append(child)
        self.records.append({'name': name, 'pid': child.pid, 'command': command, 'required': required,
                             'started_utc': utc_now(), 'monotonic_start': time.monotonic()})
        return child

    def snapshot(self):
        for child in self.children:
            try:
                for process in psutil.Process(child.pid).children(recursive=True):
                    self.descendants[process.pid] = process.create_time()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

    def ensure_alive(self, exclude=()):
        self.snapshot()
        for child, record in zip(self.children, self.records):
            if child in exclude or not record['required']:
                continue
            rc = child.poll()
            if rc is not None:
                raise ValidationError(f'{record["name"]} exited during validation (exit {rc}); see {record["name"]}.log')

    def wait_for(self, child, timeout, allow_exit=()):
        deadline = time.monotonic() + timeout
        while True:
            self.ensure_alive(exclude=(child, *allow_exit))
            rc = child.poll()
            if rc is not None:
                if rc != 0:
                    record = self.records[self.children.index(child)]
                    lines = (self.output / f'{record["name"]}.log').read_text(errors='replace').splitlines()
                    detail = lines[-1][:500] if lines else 'no diagnostic output'
                    raise ValidationError(f'{record["name"]} exited with status {rc}: {detail}; see its log')
                return
            if time.monotonic() >= deadline:
                raise ValidationError(f'Check timed out after {timeout:g} wall seconds')
            time.sleep(0.05)

    def wait_file(self, path, timeout):
        deadline = time.monotonic() + timeout
        while not Path(path).is_file():
            self.ensure_alive()
            if time.monotonic() >= deadline:
                raise ValidationError(f'Readiness timed out: {Path(path).name}')
            time.sleep(0.05)
        self.ensure_alive()
        return json.loads(Path(path).read_text())

    def command(self, name, command, timeout):
        child = self.start(name, command, required=False)
        self.wait_for(child, timeout)
        return child

    def owner_pids(self):
        self.snapshot()
        return [child.pid for child in self.children if child.poll() is None] + list(self.descendants)

    def close(self):
        if self.closed:
            return
        self.snapshot()
        # A group can outlive its leader. Never skip it merely because poll() exited.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            for child in reversed(self.children):
                if sig == signal.SIGINT:
                    # ros2 launch/run forward SIGINT. Signal the leader once,
                    # then escalate the whole group only if it does not exit.
                    if child.poll() is None:
                        with contextlib.suppress(ProcessLookupError):
                            child.send_signal(sig)
                    continue
                if not self.group_live(child.pid):
                    continue
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass
                except PermissionError:
                    # Darwin can return EPERM when a group became empty.
                    if self.group_live(child.pid):
                        raise
            for pid, birth in self.descendants.items():
                if sig == signal.SIGINT:
                    continue
                try:
                    process = psutil.Process(pid)
                    if process.create_time() == birth and process.status() != psutil.STATUS_ZOMBIE:
                        process.send_signal(sig)
                except psutil.NoSuchProcess:
                    pass
            deadline = time.monotonic() + (2 if sig != signal.SIGKILL else 1)
            while time.monotonic() < deadline:
                for child in self.children:
                    child.poll()
                if not self.remaining():
                    break
                time.sleep(0.05)
        remaining = self.remaining()
        for child, record in zip(self.children, self.records):
            child.wait(timeout=2)
            record['exit_code'] = child.returncode
            record['wall_duration_s'] = time.monotonic() - record.pop('monotonic_start')
        write_json(self.output / 'processes.json', self.records)
        self.closed = True
        if remaining:
            raise ValidationError(f'Owned processes survived cleanup: {remaining}')

    @staticmethod
    def group_live(pgid):
        for process in psutil.process_iter(['pid', 'status']):
            try:
                if process.info['status'] != psutil.STATUS_ZOMBIE and os.getpgid(process.pid) == pgid:
                    return True
            except (psutil.NoSuchProcess, ProcessLookupError, PermissionError):
                pass
        return False

    def remaining(self):
        groups = {child.pid for child in self.children}
        remaining = []
        for process in psutil.process_iter(['pid', 'status', 'create_time']):
            try:
                pid = process.pid
                same_descendant = self.descendants.get(pid) == process.info['create_time']
                if (same_descendant or os.getpgid(pid) in groups) and process.info['status'] != psutil.STATUS_ZOMBIE:
                    remaining.append(pid)
            except (psutil.NoSuchProcess, ProcessLookupError, PermissionError):
                continue
        return remaining


def require_result(path, fields):
    if not Path(path).is_file():
        raise ValidationError(f'Missing result: {Path(path).name}')
    value = json.loads(Path(path).read_text())
    if any(field not in value for field in fields):
        raise ValidationError(f'Incomplete result: {Path(path).name}')
    return value


def renderer_evidence(log_path, started_ns, owner_pids):
    """Require a fresh log held open by this run, plus actual Metal selection."""
    log_path = Path(log_path)
    if not log_path.is_file() or log_path.stat().st_mtime_ns < started_ns:
        raise ValidationError('No fresh renderer log for this run')
    owned = False
    for pid in owner_pids:
        try:
            owned |= any(Path(item.path).resolve() == log_path.resolve()
                         for item in psutil.Process(pid).open_files())
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    if not owned:
        raise ValidationError('Renderer log is not held open by this run')
    text = log_path.read_text()
    matches = re.findall(r'Metal: Requested .*?selected "([^"]+)"', text)
    if not matches or re.search(r'llvmpipe|swiftshader|software renderer', text, re.I):
        raise ValidationError('Current run did not select a hardware Metal device')
    return {'backend': 'Metal', 'device': matches[-1],
            'log_sha256': hashlib.sha256(log_path.read_bytes()).hexdigest(),
            'log_mtime_ns': log_path.stat().st_mtime_ns, 'verified_open_by_run': True}


def source_manifest():
    # Hash named files and boundaries; generated build caches are excluded.
    paths = subprocess.check_output(
        ['git', 'ls-files', '-co', '--exclude-standard'], cwd=ROOT, text=True).splitlines()
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in sorted(set(paths)) if (ROOT / name).is_file()}


def preflight(require_project=False):
    prefix = Path(os.environ.get('NATIVE_ROS_PREFIX', '/opt/homebrew/opt/ros2-jazzy'))
    expected = os.environ.get('NATIVE_ROS_EXPECTED_VERSION', '26.11.20.9')
    if platform.system() != 'Darwin' or platform.machine() != 'arm64':
        raise ValidationError('Native Metal validation requires Apple Silicon macOS')
    if sys.version_info[:2] != (3, 11) or not sys.executable.startswith('/Library/Frameworks/Python.framework/Versions/3.11/'):
        raise ValidationError('Use python.org framework Python 3.11 through tools/native.sh')
    if not (prefix / 'setup.bash').is_file() or prefix.resolve().name != expected:
        raise ValidationError(f'Expected ROS bundle {expected}; check NATIVE_ROS_PREFIX')
    if os.environ.get('LIBGL_ALWAYS_SOFTWARE', '').lower() in ('1', 'true'):
        raise ValidationError('Software rendering was requested; this check requires hardware Metal')
    if not all(shutil.which(name) for name in ('gz', 'ros2')):
        raise ValidationError('gz/ros2 are missing; source tools/native.sh')
    imports = ['rclpy', 'cv_bridge', 'cv2', 'numpy', 'yaml', 'sensor_msgs.msg']
    if require_project:
        imports.append('auto_drive_sim_interfaces.msg')
    for module in imports:
        try:
            __import__(module)
        except ImportError as error:
            raise ValidationError(f'Missing {module}; install native dependencies and rebuild packages') from error
    try:
        gazebo = subprocess.check_output(['gz', 'sim', '--version'], text=True, timeout=30).strip()
    except (subprocess.SubprocessError, OSError) as error:
        raise ValidationError('Gazebo version check failed; finish native installation first') from error
    if '8.15.0' not in gazebo:
        raise ValidationError(f'Unqualified Gazebo version: {gazebo}')
    versions = {}
    for name in ('numpy', 'opencv-python', 'PyYAML', 'psutil', 'cryptography', 'pytest'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    ros_versions = {}
    for name in ('rclpy', 'ros_gz_bridge', 'cv_bridge', 'gz_sim_vendor', 'gz_rendering_vendor', 'gz_transport_vendor'):
        manifest = prefix / 'share' / name / 'package.xml'
        ros_versions[name] = ET.parse(manifest).getroot().findtext('version') if manifest.is_file() else None
    return {'ros_packages': ros_versions, 'ros_bundle_location': str(prefix), 'architecture': platform.machine(), 'platform': platform.platform(),
            'python': platform.python_version(), 'python_executable': sys.executable,
            'ros_bundle_version': prefix.resolve().name, 'gazebo': gazebo,
            'python_packages': versions, 'network': {
                name: os.environ.get(name) for name in ('GZ_IP', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI')}}


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--project', action='store_true')
    args = parser.parse_args()
    environment = preflight(args.project)
    if args.project:
        from dataclasses import asdict
        from ament_index_python.packages import get_package_share_directory
        from auto_drive_sim.lane_controller import ControllerConfig
        from auto_drive_sim.lane_perception import CameraModel, PerceptionConfig
        environment['effective_settings'] = {
            'controller': asdict(ControllerConfig()), 'camera': asdict(CameraModel()),
            'perception': asdict(PerceptionConfig()), 'lane_strategy': 'single'}
        share = Path(get_package_share_directory('auto_drive_sim'))
        installed = {}
        assets = ROOT / 'src/auto_drive_sim'
        sources = [source for folder in ('config', 'launch', 'models', 'worlds')
                   for source in sorted((assets / folder).rglob('*'))
                   if source.is_file() and '__pycache__' not in source.parts and not source.name.startswith('.')]
        for source in sources:
            name = source.relative_to(assets).as_posix()
            actual = share / name
            if not actual.is_file() or actual.read_bytes() != source.read_bytes():
                raise ValidationError(f'Installed asset differs from {name}; rebuild native packages')
            installed[name] = hashlib.sha256(actual.read_bytes()).hexdigest()
        environment['installed_input_hashes'] = installed
    write_json(args.output, environment)
    print(json.dumps(environment, indent=2))
