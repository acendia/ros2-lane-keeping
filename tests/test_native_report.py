"""Public reports must preserve failures and reject unqualified snapshots."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from native_report import export_run, render_report


def fixture_run(tmp_path, status='passed', dirty=False):
    source = tmp_path / 'source'
    source.mkdir()
    step = {'name': 'preflight', 'status': 'passed', 'cleanup': 'passed'}
    record = {'mode': 'preflight', 'status': status, 'steps': [step], 'started_utc': '2026-11-22T00:00:00Z'}
    directory = source / 'preflight'
    directory.mkdir()
    (directory / 'validation.json').write_text(json.dumps(step))
    (directory / 'environment.json').write_text('{}')
    (source / 'validation.json').write_text(json.dumps(record))
    (source / 'metadata.json').write_text(json.dumps({'commit': 'a' * 40, 'dirty': dirty}))
    return source


@pytest.mark.parametrize('status,dirty', [('failed', False), ('passed', True), ('interrupted', False)])
def test_export_rejects_failed_or_dirty_run(tmp_path, status, dirty):
    source = fixture_run(tmp_path, status, dirty)
    with pytest.raises(ValueError, match='clean source'):
        export_run(source, tmp_path / 'public')
    assert not (tmp_path / 'public').exists()


def test_export_cannot_overwrite_old_results(tmp_path):
    source = fixture_run(tmp_path)
    output = tmp_path / 'public'
    output.mkdir()
    (output / 'sentinel').write_text('old evidence')
    with pytest.raises(FileExistsError):
        export_run(source, output)
    assert (output / 'sentinel').read_text() == 'old evidence'


def test_failure_report_does_not_claim_success():
    report = {'mode': 'camera', 'status': 'failed', 'started_utc': 'now', 'steps': [],
              'stage': 'camera', 'error': 'missing samples'}
    text = render_report(report)
    assert '**FAILED**' in text
    assert 'missing samples' in text
    assert '**PASSED**' not in text


def test_success_export_links_provenance(tmp_path):
    source = fixture_run(tmp_path)
    output = tmp_path / 'public'
    export_run(source, output)
    assert '[Source provenance](metadata.json)' in (output / 'summary.md').read_text()
    assert json.loads((output / 'metadata.json').read_text())['dirty'] is False


def test_export_rejects_missing_raw_evidence(tmp_path):
    source = fixture_run(tmp_path)
    (source / 'preflight/environment.json').unlink()
    with pytest.raises(ValueError, match='Missing raw evidence'):
        export_run(source, tmp_path / 'public')
    assert not (tmp_path / 'public').exists()
