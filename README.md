# auto-drive-sim

A camera-based lane-following simulation for ROS 2 Jazzy and Gazebo Harmonic.

## Status

Milestone 2 is in progress in Docker. The car spawns at the track start, the
Ackermann plugin receives `/car/cmd_vel`, and the camera and odometry topics
carry data. Left/right steering, stopping, and the manual straight-and-curve
drive are not verified yet, so the car is not ready for autonomous control.
See [`docs/car.md`](docs/car.md), [`docs/track.md`](docs/track.md), and
[`docs/environment.md`](docs/environment.md). The planned Ubuntu 24.04 ARM64
VM remains untested.

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

## Drive the car manually

```bash
ros2 launch auto_drive_sim manual_drive.launch.py gui:=false
ros2 run auto_drive_sim teleop
```

The teleop node uses `w/a/s/d`, space to stop, and `q` to quit. A fixed command
works without a keyboard:

```bash
ros2 run auto_drive_sim teleop --linear 0.5 --angular 0.0 --duration 2.0
```

The car model and topic contract are documented in [`docs/car.md`](docs/car.md).

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
