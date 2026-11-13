# auto-drive-sim

A camera-based lane-following simulation for ROS 2 Jazzy and Gazebo Harmonic.

## Status

Milestone 0 is in progress. The repository skeleton exists, but no Gazebo camera
check has passed yet. I will not treat the environment as validated or start the
custom road until that check is recorded in [`docs/environment.md`](docs/environment.md).

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
