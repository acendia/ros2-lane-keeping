# Environment

**Status: WORKSPACE BUILDS; CAMERA CHECK NOT VERIFIED.** The ROS workspace
builds in the Docker environment below. No Gazebo camera frames have been
received yet, so the Milestone 0 gate has not passed and Milestone 1 must not
start.

## Target environment

| Item | Planned value |
|---|---|
| OS | Ubuntu 24.04 ARM64 |
| ROS 2 | Jazzy |
| Simulator | Gazebo Harmonic |
| Python | 3.12 (Ubuntu 24.04 system Python) |
| Host | macOS on Apple Silicon |

## Working environment decision

The plan calls for an Ubuntu 24.04 ARM64 VM. During implementation I chose to
run the Linux environment in Docker on the macOS host instead, because no VM
image was available on the machine. This is a deviation from the plan. Results
measured in Docker must be labelled as such and must not be presented as VM
results.

If the Docker environment cannot render camera frames, I will stop at the
Milestone 0 gate and record the exact error rather than proceeding to the road.

## Installation

The environment is defined in [`docker/Dockerfile`](../docker/Dockerfile) and
built with:

```bash
./docker/run.sh
```

The image uses `ubuntu:24.04` (arm64) and follows the official ROS 2 Jazzy
Ubuntu instructions:

1. Add the ROS 2 apt source package (`ros2-apt-source` 1.3.0) from the
   [ros-infrastructure release](https://github.com/ros-infrastructure/ros-apt-source/releases).
2. Install `ros-jazzy-ros-base` from `packages.ros.org`.
3. Install `ros-jazzy-ros-gz`, which brings the Gazebo Harmonic libraries as
   ROS vendor packages, plus `ros-jazzy-gz-tools-vendor` and
   `ros-jazzy-gz-sim-vendor`.

I did not install `gz-harmonic` from `packages.osrfoundation.org` on top of the
vendor packages. The [ROS/Gazebo pairing documentation](https://gazebosim.org/docs/harmonic/ros_installation/)
recommends `ros-${ROS_DISTRO}-ros-gz` as the supported combination, and mixing
both sources risks package conflicts.

The build is run with:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Camera feasibility check

Not yet run. The check must demonstrate:

- Gazebo starts headlessly.
- A test camera publishes actual image frames through ROS 2.
- The image dimensions and frame rate are usable.

## Observed results

Measured on 2026-11-13 in the Docker container.

| Item | Observed value |
|---|---|
| Architecture | aarch64 |
| Ubuntu version | Ubuntu 24.04.5 LTS |
| ROS 2 distribution | Jazzy, `ros-jazzy-ros-base` 0.11.0, `ros-jazzy-ros2cli` 0.32.13 |
| Gazebo version | Gazebo Sim 8.15.0 (Harmonic vendor packages) |
| Python version | 3.12.3 |
| Camera topic | not measured |
| Image dimensions | not measured |
| Frame rate | not measured |
| Rendering backend | not measured |

## Build results

```text
Starting >>> auto_drive_sim_interfaces
Finished <<< auto_drive_sim_interfaces [4.60s]
Starting >>> auto_drive_sim
Finished <<< auto_drive_sim [0.84s]

Summary: 2 packages finished [5.52s]
```

`ros2 pkg list` reports both `auto_drive_sim` and
`auto_drive_sim_interfaces`, and
`ros2 interface show auto_drive_sim_interfaces/msg/LaneEstimate` prints the
expected fields.

## Build issues and revisions

- Initial `colcon build` failed because `catkin_pkg` rejected the maintainer
  email `evangelos@localhost`. I replaced it with the reserved placeholder
  `evangelos.kostakis@example.com` in both `package.xml` files and `setup.py`.
  The build then completed.

## Rendering issues and limitations

None recorded yet.

## Commands run

```bash
docker build --progress=plain -t auto-drive-sim:jazzy-harmonic -f docker/Dockerfile .
./docker/run.sh bash -lc 'ros2 --version'                      # not a valid Jazzy command
./docker/run.sh bash -lc 'gz sim --version ... colcon build --symlink-install'
```

`ros2 --version` failed with `unrecognized arguments: --version`; Jazzy reports
versions through package metadata instead, which is what the table above uses.
