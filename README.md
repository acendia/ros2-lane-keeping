# auto-drive-sim

A camera-based lane-following simulation for ROS 2 Jazzy and Gazebo Harmonic.

## Status

The Milestone 0 camera gate passed in Docker: Gazebo Harmonic runs headlessly
and publishes real 640 × 360 camera frames through ROS 2 at about 14 Hz. The
planned Ubuntu 24.04 ARM64 VM has not been tested, and Milestone 1 has not
started. See [`docs/environment.md`](docs/environment.md) for the measured
result and the documented Docker deviation.

## Repository layout

```text
auto-drive-sim/
├── src/
│   ├── auto_drive_sim/                 # Python nodes, config, launch, models, worlds
│   └── auto_drive_sim_interfaces/      # LaneEstimate.msg
├── tools/                              # Track generator and scenario tools
├── tests/                              # Geometry, perception, control, integration tests
└── docs/                               # Setup, architecture, experiments, results
```

## Prerequisites

- Ubuntu 24.04 ARM64 (the plan's target) or the documented Docker development
  environment described in `docs/environment.md`
- ROS 2 Jazzy
- Gazebo Harmonic

Installation and verification commands are recorded in
[`docs/environment.md`](docs/environment.md) once they have been run.

## Build and launch

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch auto_drive_sim demo.launch.py gui:=false
```

The launch file does not exist yet; this is the intended interface.

For the documented Docker development environment:

```bash
./docker/run.sh                                  # interactive shell
./docker/run.sh bash -lc 'colcon build --symlink-install'
```

## License

MIT. See [`LICENSE`](LICENSE).
