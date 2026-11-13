# auto-drive-sim

A camera-based lane-following simulation for ROS 2 Jazzy and Gazebo Harmonic.

## Status

Milestone 1 is complete in Docker. Gazebo loads an original 134.25 m oval with
a 3.5 m lane, and four camera views show both continuous white boundaries on
the straight and both curves. The planned Ubuntu 24.04 ARM64 VM has not been
tested, and the car does not exist yet. See
[`docs/environment.md`](docs/environment.md) and [`docs/track.md`](docs/track.md)
for measured results and the documented Docker deviation.

## Repository layout

```text
auto-drive-sim/
├── src/
│   ├── auto_drive_sim/                 # Python nodes, config, launch, models, worlds
│   └── auto_drive_sim_interfaces/      # LaneEstimate.msg
├── tools/                              # Track generator and scenario tools
├── tests/                              # Geometry, perception, control, integration tests
└── docs/                               # Setup, track, architecture, experiments, results
```

## Prerequisites

- Ubuntu 24.04 ARM64 (the plan's target) or the documented Docker development
  environment described in `docs/environment.md`
- ROS 2 Jazzy
- Gazebo Harmonic

Installation and verification commands are recorded in
[`docs/environment.md`](docs/environment.md).

## Build and preview the track

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch auto_drive_sim track_preview.launch.py gui:=false
```

`gui:=true` starts the Gazebo client on a machine with a display.

For the documented Docker development environment:

```bash
./docker/run.sh                                  # interactive shell
./docker/run.sh bash -lc 'colcon build --symlink-install'
```

## Regenerate the track

```bash
python3 tools/generate_track.py
python3 -m pytest -q tests/test_track_geometry.py
```

The generator and the committed assets are checked for byte-identical
regeneration. See [`docs/track.md`](docs/track.md) for the dimensions,
coordinate frame, and camera verification.

## Test the track camera views

```bash
./docker/run.sh bash -lc 'tools/run_road_camera_check.sh'
```

This starts the headless world, bridges four cameras, and writes frames to
`artifacts/road_check/`.

## License

MIT. See [`LICENSE`](LICENSE).
