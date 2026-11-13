#!/usr/bin/env python3
"""Generate the oval track meshes and evaluation centerline.

The generator is deliberately dependency-light: Python, PyYAML, and the
standard library. Running it twice from the same configuration must produce
byte-identical assets.
"""

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / 'src/auto_drive_sim/config/track.yaml'
DEFAULT_OUTPUT = REPO_ROOT / 'src/auto_drive_sim/models/road'


@dataclass(frozen=True)
class TrackConfig:
    """Validated track geometry parameters."""

    track_version: int
    straight_length_m: float
    curve_radius_m: float
    lane_width_m: float
    marking_width_m: float
    shoulder_width_m: float
    sample_spacing_m: float
    asphalt_height_m: float
    marking_height_m: float
    asphalt_color_rgb: tuple
    marking_color_rgb: tuple
    start_s_m: float
    start_z_m: float


@dataclass(frozen=True)
class Sample:
    """One centerline sample in the world frame."""

    s_m: float
    x_m: float
    y_m: float
    yaw_rad: float
    curvature_1_per_m: float


def _positive(value, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be a positive finite number')
    return value


def _color(value, name: str) -> tuple:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{name} must be a list of three numbers')
    color = tuple(float(channel) for channel in value)
    if any(not 0.0 <= channel <= 1.0 for channel in color):
        raise ValueError(f'{name} channels must be in [0, 1]')
    return color


def load_config(path: Path = DEFAULT_CONFIG) -> TrackConfig:
    """Read and validate the track configuration."""
    with path.open('r', encoding='utf-8') as stream:
        raw = yaml.safe_load(stream)

    track = raw['track']
    start = raw['start_pose']
    config = TrackConfig(
        track_version=int(raw['track_version']),
        straight_length_m=_positive(
            track['straight_length_m'], 'straight_length_m'),
        curve_radius_m=_positive(track['curve_radius_m'], 'curve_radius_m'),
        lane_width_m=_positive(track['lane_width_m'], 'lane_width_m'),
        marking_width_m=_positive(track['marking_width_m'], 'marking_width_m'),
        shoulder_width_m=float(track['shoulder_width_m']),
        sample_spacing_m=_positive(
            track['sample_spacing_m'], 'sample_spacing_m'),
        asphalt_height_m=float(track['asphalt_height_m']),
        marking_height_m=float(track['marking_height_m']),
        asphalt_color_rgb=_color(
            track['asphalt_color_rgb'], 'asphalt_color_rgb'),
        marking_color_rgb=_color(
            track['marking_color_rgb'], 'marking_color_rgb'),
        start_s_m=float(start['s_m']),
        start_z_m=float(start['z_m']),
    )

    if config.marking_width_m >= config.lane_width_m:
        raise ValueError('marking_width_m must be smaller than lane_width_m')
    if config.shoulder_width_m < 0.0:
        raise ValueError('shoulder_width_m must not be negative')
    if config.marking_height_m <= config.asphalt_height_m:
        raise ValueError('marking_height_m must exceed asphalt_height_m')

    length = track_length(config)
    if not 0.0 <= config.start_s_m < length:
        raise ValueError('start_pose.s_m must be inside the track')
    return config


def track_length(config: TrackConfig) -> float:
    """Return the exact centerline length."""
    return 2.0 * config.straight_length_m + 2.0 * math.pi * config.curve_radius_m


def sample_centerline(config: TrackConfig) -> list:
    """Sample the clockwise oval centerline without duplicating joints."""
    straight = config.straight_length_m
    radius = config.curve_radius_m
    spacing = config.sample_spacing_m
    half_straight = straight / 2.0
    curve_length = math.pi * radius

    samples = []
    s_m = 0.0

    # Top straight, from west to east.
    count = max(1, round(straight / spacing))
    step = straight / count
    for index in range(count):
        x_m = -half_straight + index * step
        samples.append(Sample(s_m, x_m, radius, 0.0, 0.0))
        s_m += step

    # East semicircle, clockwise from north to south.
    count = max(1, round(curve_length / spacing))
    step = curve_length / count
    for index in range(count):
        angle = math.pi / 2.0 - (index * step) / radius
        x_m = half_straight + radius * math.cos(angle)
        y_m = radius * math.sin(angle)
        samples.append(Sample(s_m, x_m, y_m, angle - math.pi / 2.0,
                              -1.0 / radius))
        s_m += step

    # Bottom straight, from east to west.
    count = max(1, round(straight / spacing))
    step = straight / count
    for index in range(count):
        x_m = half_straight - index * step
        samples.append(Sample(s_m, x_m, -radius, math.pi, 0.0))
        s_m += step

    # West semicircle, clockwise from south to north.
    count = max(1, round(curve_length / spacing))
    step = curve_length / count
    for index in range(count):
        angle = -math.pi / 2.0 - (index * step) / radius
        x_m = -half_straight + radius * math.cos(angle)
        y_m = radius * math.sin(angle)
        samples.append(Sample(s_m, x_m, y_m, angle - math.pi / 2.0,
                              -1.0 / radius))
        s_m += step

    return samples


def pose_at_s(config: TrackConfig, samples: list, s_m: float) -> tuple:
    """Return the nearest centerline sample to the requested arc length."""
    target = s_m % track_length(config)
    return min(samples, key=lambda sample: abs(sample.s_m - target))


def ribbon_vertices(samples: list, offset_m: float, half_width_m: float,
                    z_m: float) -> list:
    """Build a closed ribbon as left/right vertex pairs around the centerline."""
    vertices = []
    for sample in samples:
        left_x = -math.sin(sample.yaw_rad)
        left_y = math.cos(sample.yaw_rad)
        outer = offset_m + half_width_m
        inner = offset_m - half_width_m
        vertices.append((
            sample.x_m + left_x * outer,
            sample.y_m + left_y * outer,
            z_m,
        ))
        vertices.append((
            sample.x_m + left_x * inner,
            sample.y_m + left_y * inner,
            z_m,
        ))
    return vertices


def ribbon_faces(sample_count: int) -> list:
    """Triangulate a closed ribbon whose vertices are left/right pairs."""
    faces = []
    for index in range(sample_count):
        following = (index + 1) % sample_count
        left = 2 * index + 1
        right = 2 * index + 2
        next_left = 2 * following + 1
        next_right = 2 * following + 2
        faces.append((left, right, next_right))
        faces.append((left, next_right, next_left))
    return faces


def _number(value: float) -> str:
    if abs(value) < 0.5e-6:
        value = 0.0
    return f'{value:.6f}'


def _write_mtl(path: Path, config: TrackConfig) -> None:
    asphalt = ' '.join(_number(channel) for channel in config.asphalt_color_rgb)
    marking = ' '.join(_number(channel) for channel in config.marking_color_rgb)
    content = (
        '# Generated by tools/generate_track.py. Do not edit by hand.\n'
        'newmtl asphalt\n'
        'Ka 0.150000 0.150000 0.150000\n'
        f'Kd {asphalt}\n'
        'Ks 0.000000 0.000000 0.000000\n'
        'd 1.000000\n'
        'illum 1\n'
        '\n'
        'newmtl lane_marking\n'
        'Ka 0.900000 0.900000 0.900000\n'
        f'Kd {marking}\n'
        'Ks 0.000000 0.000000 0.000000\n'
        'd 1.000000\n'
        'illum 1\n'
    )
    path.write_text(content, encoding='utf-8', newline='\n')


def _write_obj(path: Path, name: str, vertices: list, faces: list,
               material: str) -> None:
    lines = [
        '# Generated by tools/generate_track.py. Do not edit by hand.',
        'mtllib road.mtl',
        f'o {name}',
    ]
    for x_m, y_m, z_m in vertices:
        lines.append(f'v {_number(x_m)} {_number(y_m)} {_number(z_m)}')
    lines.append('vn 0.000000 0.000000 1.000000')
    lines.append(f'usemtl {material}')
    for first, second, third in faces:
        lines.append(f'f {first}//1 {second}//1 {third}//1')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8', newline='\n')


def _write_centerline(path: Path, samples: list) -> None:
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['s_m', 'x_m', 'y_m', 'yaw_rad', 'curvature_1_per_m'])
        for sample in samples:
            writer.writerow([
                _number(sample.s_m),
                _number(sample.x_m),
                _number(sample.y_m),
                _number(sample.yaw_rad),
                _number(sample.curvature_1_per_m),
            ])


def _write_model_sdf(path: Path) -> None:
    content = """<?xml version="1.0" ?>
<!-- Generated by tools/generate_track.py. Do not edit by hand. -->
<sdf version="1.9">
  <model name="road">
    <static>true</static>
    <link name="road_link">
      <visual name="asphalt">
        <geometry>
          <mesh>
            <uri>model://road/meshes/asphalt.obj</uri>
          </mesh>
        </geometry>
        <cast_shadows>false</cast_shadows>
      </visual>
      <visual name="left_boundary">
        <geometry>
          <mesh>
            <uri>model://road/meshes/left_boundary.obj</uri>
          </mesh>
        </geometry>
        <cast_shadows>false</cast_shadows>
      </visual>
      <visual name="right_boundary">
        <geometry>
          <mesh>
            <uri>model://road/meshes/right_boundary.obj</uri>
          </mesh>
        </geometry>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>
"""
    path.write_text(content, encoding='utf-8', newline='\n')


def _write_model_config(path: Path) -> None:
    content = """<?xml version="1.0"?>
<model>
  <name>Auto Drive Sim Road</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Evangelos Kostakis</name>
    <email>evangelos.kostakis@example.com</email>
  </author>
  <description>Original oval lane generated by tools/generate_track.py.</description>
</model>
"""
    path.write_text(content, encoding='utf-8', newline='\n')


def generate_assets(config: TrackConfig, output_dir: Path) -> dict:
    """Write all track assets and return a summary."""
    meshes_dir = output_dir / 'meshes'
    meshes_dir.mkdir(parents=True, exist_ok=True)

    samples = sample_centerline(config)
    sample_count = len(samples)
    left_center = config.lane_width_m / 2.0 + config.marking_width_m / 2.0
    marking_half = config.marking_width_m / 2.0
    asphalt_half = config.lane_width_m / 2.0 + config.shoulder_width_m

    asphalt_vertices = ribbon_vertices(
        samples, 0.0, asphalt_half, config.asphalt_height_m)
    left_vertices = ribbon_vertices(
        samples, left_center, marking_half, config.marking_height_m)
    right_vertices = ribbon_vertices(
        samples, -left_center, marking_half, config.marking_height_m)
    faces = ribbon_faces(sample_count)

    _write_mtl(meshes_dir / 'road.mtl', config)
    _write_obj(meshes_dir / 'asphalt.obj', 'asphalt',
               asphalt_vertices, faces, 'asphalt')
    _write_obj(meshes_dir / 'left_boundary.obj', 'left_boundary',
               left_vertices, faces, 'lane_marking')
    _write_obj(meshes_dir / 'right_boundary.obj', 'right_boundary',
               right_vertices, faces, 'lane_marking')
    _write_centerline(output_dir / 'centerline.csv', samples)
    _write_model_sdf(output_dir / 'model.sdf')
    _write_model_config(output_dir / 'model.config')

    start = pose_at_s(config, samples, config.start_s_m)
    summary = {
        'track_version': config.track_version,
        'centerline_length_m': round(track_length(config), 6),
        'centerline_samples': sample_count,
        'lane_width_m': config.lane_width_m,
        'straight_length_m': config.straight_length_m,
        'curve_radius_m': config.curve_radius_m,
        'start_pose': {
            'x_m': round(start.x_m, 6),
            'y_m': round(start.y_m, 6),
            'z_m': config.start_z_m,
            'yaw_rad': round(start.yaw_rad, 6),
        },
    }
    (output_dir / 'track_meta.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8', newline='\n')
    return summary


def main() -> int:
    """Generate assets from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    config = load_config(args.config)
    summary = generate_assets(config, args.output)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
