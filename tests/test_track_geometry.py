"""Geometry and reproducibility tests for the generated oval track."""

import importlib.util
import math
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO_ROOT / 'src/auto_drive_sim/config/track.yaml'
ASSET_DIR = REPO_ROOT / 'src/auto_drive_sim/models/road'
MARKINGS_CONFIG_PATH = (
    REPO_ROOT / 'src/auto_drive_sim/config/track_markings.yaml')
MARKINGS_ASSET_DIR = REPO_ROOT / 'src/auto_drive_sim/models/road_markings'

_SPEC = importlib.util.spec_from_file_location(
    'generate_track', REPO_ROOT / 'tools' / 'generate_track.py')
generate_track = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(generate_track)


@pytest.fixture(scope='module')
def config():
    """Load the checked-in track configuration."""
    return generate_track.load_config(CONFIG_PATH)


@pytest.fixture(scope='module')
def samples(config):
    """Sample the centerline once for the module."""
    return generate_track.sample_centerline(config)


def test_centerline_length(config):
    """The sampled centerline must match the stadium perimeter."""
    expected = 2.0 * 20.0 + 2.0 * math.pi * 15.0
    assert generate_track.track_length(config) == pytest.approx(expected)


def test_centerline_closure(config, samples):
    """The last sample must be one step before the first sample."""
    first = samples[0]
    last = samples[-1]
    gap = math.hypot(first.x_m - last.x_m, first.y_m - last.y_m)
    assert gap == pytest.approx(config.sample_spacing_m, abs=0.02)
    assert first.s_m == pytest.approx(0.0)
    assert samples[-1].s_m < generate_track.track_length(config)


def test_start_pose_matches_first_sample(config, samples):
    """The start pose must sit on the beginning of the top straight."""
    assert samples[0].x_m == pytest.approx(-10.0)
    assert samples[0].y_m == pytest.approx(15.0)
    assert samples[0].yaw_rad == pytest.approx(0.0)


def test_lane_width_between_inner_marking_edges(config, samples):
    """The clear lane width must be exactly 3.5 m."""
    marking_half = config.marking_width_m / 2.0
    marking_center = config.lane_width_m / 2.0 + marking_half
    left = generate_track.ribbon_vertices(
        samples, marking_center, marking_half, config.marking_height_m)
    right = generate_track.ribbon_vertices(
        samples, -marking_center, marking_half, config.marking_height_m)

    inner_left = left[1]
    inner_right = right[0]
    width = math.hypot(
        inner_left[0] - inner_right[0], inner_left[1] - inner_right[1])
    assert width == pytest.approx(config.lane_width_m)


def test_asphalt_bounds_and_heights(config, samples):
    """The asphalt must extend to the expected oval bounds at one height."""
    half_width = config.lane_width_m / 2.0 + config.shoulder_width_m
    vertices = generate_track.ribbon_vertices(
        samples, 0.0, half_width, config.asphalt_height_m)
    x_values = [vertex[0] for vertex in vertices]
    y_values = [vertex[1] for vertex in vertices]
    z_values = {vertex[2] for vertex in vertices}
    tolerance = config.sample_spacing_m

    assert min(x_values) == pytest.approx(-25.0 - half_width, abs=tolerance)
    assert max(x_values) == pytest.approx(25.0 + half_width, abs=tolerance)
    assert min(y_values) == pytest.approx(-15.0 - half_width, abs=tolerance)
    assert max(y_values) == pytest.approx(15.0 + half_width, abs=tolerance)
    assert z_values == {config.asphalt_height_m}


def test_straight_and_curve_curvature(config, samples):
    """Every sample must be either straight or a constant-radius right turn."""
    straight_count = sum(
        1 for sample in samples if sample.curvature_1_per_m == 0.0)
    curve_count = sum(
        1 for sample in samples
        if sample.curvature_1_per_m == pytest.approx(
            -1.0 / config.curve_radius_m))
    assert straight_count + curve_count == len(samples)
    assert straight_count > 0
    assert curve_count > 0


def test_regeneration_is_byte_identical(config, tmp_path):
    """Two independent generations must produce identical bytes."""
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    generate_track.generate_assets(config, first)
    generate_track.generate_assets(config, second)

    first_files = sorted(path.relative_to(first) for path in first.rglob('*')
                         if path.is_file())
    second_files = sorted(path.relative_to(second) for path in second.rglob('*')
                          if path.is_file())
    assert first_files == second_files
    for relative in first_files:
        assert (first / relative).read_bytes() == \
            (second / relative).read_bytes()


def test_committed_assets_are_current(config, tmp_path):
    """The checked-in assets must match a fresh generation."""
    generate_track.generate_assets(config, tmp_path)
    generated_files = sorted(
        path.relative_to(tmp_path) for path in tmp_path.rglob('*')
        if path.is_file())

    for relative in generated_files:
        committed = ASSET_DIR / relative
        assert committed.is_file(), f'missing committed asset {relative}'
        assert committed.read_bytes() == (tmp_path / relative).read_bytes(), \
            f'committed asset {relative} is stale'


@pytest.fixture(scope='module')
def markings_config():
    """Load the dashed and worn track configuration."""
    return generate_track.load_config(MARKINGS_CONFIG_PATH)


def test_dashed_marking_pattern(markings_config):
    """Dashes must alternate with gaps and split every third dash."""
    segments = generate_track.marking_segments(markings_config)
    assert len(segments) == 48
    assert segments[0].start_s_m == pytest.approx(0.0)
    assert segments[0].end_s_m == pytest.approx(3.0)
    assert segments[1].start_s_m == pytest.approx(3.75)
    assert not segments[0].worn
    assert segments[2].start_s_m == pytest.approx(7.5)
    assert segments[2].worn
    assert segments[3].start_s_m == pytest.approx(9.09)
    assert sum(1 for segment in segments if segment.worn) == 24
    for segment in segments:
        assert 0.0 < segment.end_s_m - segment.start_s_m <= 3.0


def test_markings_meshes_reference_valid_vertices(markings_config):
    """Every face index must point at a vertex that exists in the mesh."""
    for mesh in sorted((MARKINGS_ASSET_DIR / 'meshes').glob('*.obj')):
        vertex_count = 0
        max_index = 0
        for line in mesh.read_text(encoding='utf-8').splitlines():
            if line.startswith('v '):
                vertex_count += 1
            elif line.startswith('f '):
                for token in line.split()[1:]:
                    max_index = max(max_index, int(token.split('/')[0]))
        assert max_index <= vertex_count, f'{mesh.name} has bad face indices'


def test_markings_regeneration_is_byte_identical(markings_config, tmp_path):
    """Two dashed track generations must produce identical bytes."""
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    generate_track.generate_assets(markings_config, first)
    generate_track.generate_assets(markings_config, second)

    first_files = sorted(path.relative_to(first) for path in first.rglob('*')
                         if path.is_file())
    second_files = sorted(path.relative_to(second) for path in second.rglob('*')
                          if path.is_file())
    assert first_files == second_files
    for relative in first_files:
        assert (first / relative).read_bytes() == \
            (second / relative).read_bytes()


def test_committed_markings_assets_are_current(markings_config, tmp_path):
    """The checked-in dashed track assets must match a fresh generation."""
    generate_track.generate_assets(markings_config, tmp_path)
    generated_files = sorted(
        path.relative_to(tmp_path) for path in tmp_path.rglob('*')
        if path.is_file())

    for relative in generated_files:
        committed = MARKINGS_ASSET_DIR / relative
        assert committed.is_file(), f'missing committed asset {relative}'
        assert committed.read_bytes() == (tmp_path / relative).read_bytes(), \
            f'committed asset {relative} is stale'
