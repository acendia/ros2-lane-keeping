"""Failures must never be summarized using a previous run's passing files."""
import os
import sys
import pytest
from pathlib import Path
import subprocess


@pytest.mark.skipif(sys.platform == 'darwin', reason='Linux runner; native macOS uses native_check.py')
def test_failed_evaluator_cannot_reuse_previous_result(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    previous = tmp_path / 'results' / 'center'
    previous.mkdir(parents=True)
    (previous / 'result.json').write_text('{"lap_complete": true}')
    mock_bin = tmp_path / 'bin'
    mock_bin.mkdir()
    for name, body in [('ros2', 'exit 7'), ('sleep', 'exit 0')]:
        executable = mock_bin / name
        executable.write_text('#!/bin/bash\n' + body + '\n')
        executable.chmod(0o755)
    script = (repo / 'tools/run_scenarios.sh').read_text()
    script = script.replace('repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"',
                            f'repo_dir="{repo}"')
    script = script.replace('source /opt/ros/jazzy/setup.bash', ':')
    script = script.replace('source "$repo_dir/install/setup.bash"', ':')
    runner = tmp_path / 'runner.sh'
    runner.write_text(script)
    env = dict(os.environ, PATH=str(mock_bin) + ':' + os.environ['PATH'],
               RESULTS_DIR=str(previous.parent))
    result = subprocess.run(['bash', str(runner), 'center'], env=env,
                            capture_output=True, text=True, timeout=20)
    assert result.returncode != 0
    assert 'evaluation failed:' in result.stderr
    assert not list(previous.parent.rglob('summary.md'))
    assert (previous / 'result.json').read_text() == '{"lap_complete": true}'
