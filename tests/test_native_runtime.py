"""Failure, isolation, and provenance checks without a live simulator."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import psutil
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import native_runtime as runtime
import native_check as check


def test_cleanup_reaches_grandchild_after_parent_exits(tmp_path):
    pid_file = tmp_path / 'grandchild.pid'
    processes = runtime.Processes(tmp_path)
    script = ('import subprocess,sys; from pathlib import Path; '
              'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"]); '
              f'Path({str(pid_file)!r}).write_text(str(p.pid))')
    parent = processes.start('parent', [sys.executable, '-c', script])
    try:
        parent.wait(timeout=5)
        grandchild = int(pid_file.read_text())
        assert psutil.pid_exists(grandchild)
    finally:
        processes.close()
    assert not processes.remaining()
    if psutil.pid_exists(grandchild):
        assert psutil.Process(grandchild).status() == psutil.STATUS_ZOMBIE


def test_cleanup_does_not_terminate_unrelated_process(tmp_path):
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    try:
        processes = runtime.Processes(tmp_path)
        processes.start('owned', [sys.executable, '-c', 'import time; time.sleep(60)'])
        processes.close()
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=5)


def test_child_crash_interrupts_readiness_immediately(tmp_path):
    processes = runtime.Processes(tmp_path)
    processes.start('gazebo', [sys.executable, '-c', 'raise SystemExit(7)'])
    started = time.monotonic()
    try:
        with pytest.raises(runtime.ValidationError, match='gazebo exited'):
            processes.wait_file(tmp_path / 'never.json', 10)
        assert time.monotonic() - started < 3
    finally:
        processes.close()


def test_readiness_and_command_timeouts_fail(tmp_path):
    processes = runtime.Processes(tmp_path)
    try:
        with pytest.raises(runtime.ValidationError, match='Readiness timed out'):
            processes.wait_file(tmp_path / 'missing.json', 0.1)
        with pytest.raises(runtime.ValidationError, match='timed out'):
            processes.command('hung', [sys.executable, '-c', 'import time; time.sleep(60)'], .1)
    finally:
        processes.close()
    assert not processes.remaining()


def test_domain_and_renderer_lock_reject_concurrent_check():
    with runtime.RunLock(73):
        with pytest.raises(runtime.ValidationError, match='Another native check'):
            with runtime.RunLock(73):
                pass
    with runtime.RunLock(73):
        pass


def test_result_file_must_be_present_and_complete(tmp_path):
    path = tmp_path / 'result.json'
    with pytest.raises(runtime.ValidationError, match='Missing result'):
        runtime.require_result(path, ('samples',))
    path.write_text('{}')
    with pytest.raises(runtime.ValidationError, match='Incomplete result'):
        runtime.require_result(path, ('samples',))


def test_renderer_requires_current_owned_log_and_selected_device(tmp_path, monkeypatch):
    log = tmp_path / 'ogre2.log'
    started = time.time_ns()
    log.write_text('Metal: Requested "(default)", selected "Apple M1 Pro (system default)"\n')
    os.utime(log, ns=(started + 1000000, started + 1000000))
    monkeypatch.setattr(runtime.psutil, 'Process', lambda _: SimpleNamespace(
        open_files=lambda: [SimpleNamespace(path=str(log))]))
    evidence = runtime.renderer_evidence(log, started, [123])
    assert evidence['backend'] == 'Metal'
    assert 'Apple M1 Pro' in evidence['device']
    with pytest.raises(runtime.ValidationError, match='fresh'):
        runtime.renderer_evidence(log, time.time_ns() + 1000000000, [123])
    with pytest.raises(runtime.ValidationError, match='held open'):
        runtime.renderer_evidence(log, started, [])
    for text in ('Installing plugin: Metal RenderSystem', 'llvmpipe software renderer',
                 'OpenGL Rendering Subsystem'):
        log.write_text(text)
        os.utime(log, ns=(started + 1000000, started + 1000000))
        with pytest.raises(runtime.ValidationError, match='hardware Metal'):
            runtime.renderer_evidence(log, started, [123])


def test_preflight_reports_missing_bundle_and_wrong_python(tmp_path, monkeypatch):
    monkeypatch.setattr(runtime.platform, 'system', lambda: 'Darwin')
    monkeypatch.setattr(runtime.platform, 'machine', lambda: 'arm64')
    monkeypatch.setattr(runtime.sys, 'version_info', (3, 11))
    monkeypatch.setattr(runtime.sys, 'executable', '/wrong/python')
    with pytest.raises(runtime.ValidationError, match='framework Python'):
        runtime.preflight()
    monkeypatch.setattr(runtime.sys, 'executable', '/Library/Frameworks/Python.framework/Versions/3.11/bin/python3')
    monkeypatch.setenv('NATIVE_ROS_PREFIX', str(tmp_path))
    with pytest.raises(runtime.ValidationError, match='Expected ROS bundle'):
        runtime.preflight()


def test_named_source_hashes_change_when_a_file_changes(tmp_path, monkeypatch):
    (tmp_path / 'one.py').write_text('first')
    monkeypatch.setattr(runtime, 'ROOT', tmp_path)
    monkeypatch.setattr(runtime.subprocess, 'check_output', lambda *a, **kw: 'one.py\n')
    first = runtime.source_manifest()
    (tmp_path / 'one.py').write_text('second')
    assert runtime.source_manifest() != first
    assert set(first) == {'one.py'}


def test_failure_and_interrupt_are_written_to_manifest(tmp_path, monkeypatch):
    args = SimpleNamespace(mode='preflight', domain=74, duration=1, repetitions=1,
                           startup_timeout=1, scenario_timeout=1, require_clean=False)
    for error, code, status in ((RuntimeError('fake import failure'), 1, 'failed'),
                                 (KeyboardInterrupt(), 130, 'interrupted')):
        out = tmp_path / status
        out.mkdir()
        def fail(*a, **kw):
            raise error
        monkeypatch.setattr(check, 'scoped_step', fail)
        assert check.execute(args, out) == code
        record = json.loads((out / 'validation.json').read_text())
        assert record['status'] == status
        assert record['stage'] == 'preflight'
        assert 'error' in record


def test_scoped_failure_still_cleans_processes_and_records_failure(tmp_path):
    report = {'steps': []}
    def fail(processes, output):
        processes.start('child', [sys.executable, '-c', 'import time; time.sleep(60)'])
        raise RuntimeError('fake evaluator crash')
    with pytest.raises(RuntimeError, match='evaluator crash'):
        check.scoped_step(tmp_path / 'step', fail, report, 'step')
    record = json.loads((tmp_path / 'step/validation.json').read_text())
    assert record['status'] == 'failed'
    assert record['cleanup'] == 'passed'


def test_preflight_rejects_software_and_missing_import(tmp_path, monkeypatch):
    monkeypatch.setattr(runtime.platform, 'system', lambda: 'Darwin')
    monkeypatch.setattr(runtime.platform, 'machine', lambda: 'arm64')
    monkeypatch.setattr(runtime.sys, 'version_info', (3, 11))
    monkeypatch.setattr(runtime.sys, 'executable', '/Library/Frameworks/Python.framework/Versions/3.11/bin/python3')
    prefix = tmp_path / '26.11.20.9'
    prefix.mkdir()
    (prefix / 'setup.bash').touch()
    monkeypatch.setenv('NATIVE_ROS_PREFIX', str(prefix))
    monkeypatch.setenv('LIBGL_ALWAYS_SOFTWARE', '1')
    with pytest.raises(runtime.ValidationError, match='Software rendering'):
        runtime.preflight()
    monkeypatch.delenv('LIBGL_ALWAYS_SOFTWARE')
    monkeypatch.setattr(runtime.shutil, 'which', lambda _: '/stub')
    def missing(name):
        if name == 'cv_bridge':
            raise ModuleNotFoundError(name)
        return SimpleNamespace()
    monkeypatch.setattr(runtime, '__import__', missing, raising=False)
    with pytest.raises(runtime.ValidationError, match='Missing cv_bridge'):
        runtime.preflight()


def test_interrupt_forwarding_does_not_double_signal_grandchild(tmp_path):
    ready = tmp_path / 'ready'
    signals = tmp_path / 'signals'
    child_code = (
        'import signal,time; from pathlib import Path; '
        f'path=Path({str(signals)!r}); '
        'signal.signal(signal.SIGINT, lambda *a: path.write_text(path.read_text()+"I")); '
        f'Path({str(ready)!r}).touch(); '
        'time.sleep(60)')
    parent_code = (
        'import subprocess,sys,signal,time; '
        f'p=subprocess.Popen([sys.executable,"-c",{child_code!r}]); '
        'signal.signal(signal.SIGINT, lambda *a: p.send_signal(signal.SIGINT)); '
        'time.sleep(60)')
    signals.write_text('')
    processes = runtime.Processes(tmp_path)
    processes.start('forwarder', [sys.executable, '-c', parent_code])
    end = time.monotonic() + 5
    try:
        while not ready.exists() and time.monotonic() < end:
            time.sleep(.02)
        assert ready.exists()
    finally:
        processes.close()
    assert signals.read_text() == 'I'
