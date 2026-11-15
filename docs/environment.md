# Environment

**Status: DOCKER PASSES THE CAMERA GATE AND RELEASE TARGET. THE LIMA VM
PRODUCES VALID CAMERA FRAMES BUT IS TOO SLOW FOR THE RELEASE TARGET.** The VM
builds, passes the 50 tests present at that revision, and renders 640 × 360
frames at 4.5–5.4 Hz. That rate
is not enough for the 0.3 s stale rule, so all three VM scenarios fail. See
[VM validation](#vm-validation-lima).

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

The Docker environment did render camera frames, so the Milestone 0 gate was
not blocked. I later created a native Ubuntu 24.04 ARM64 VM with Lima 2.3.0.
That VM builds, tests, and renders, but its 4.5–5.4 Hz frame rate is not
usable for the control loop. Docker remains the only environment that meets
the release target.

**Support policy:** Docker is the supported release environment. The native
VM build and check instructions are kept for reference, but VM result claims
must not be presented as a passing release. Results from any new environment
need the full three-scenario run before they count.

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

The check uses [`tests/gazebo/camera_check.sdf`](../tests/gazebo/camera_check.sdf),
a static scene with a white ground stripe and blue, green, and red boxes in
front of a 1.3 m high camera. The camera renders 640 × 360 at 15 Hz.

Run it with:

```bash
tools/run_camera_check.sh
```

The script starts Gazebo headlessly, bridges the Gazebo image topic to ROS 2,
runs [`tools/camera_probe.py`](../tools/camera_probe.py), and saves one frame.
The probe exits non-zero when frames are missing, the dimensions are wrong, or
the image is nearly uniform.

### Gate result

Measured on 2026-11-13 in the Docker container:

| Check | Result |
|---|---|
| Gazebo starts headlessly | Yes, `gz sim -s -r --headless-rendering` |
| Camera publishes through ROS 2 | Yes, 139 frames in a 10 s probe |
| Gazebo image topic | `/camera_check/image` |
| ROS image topic | `/camera_check/image` |
| Camera info topic | `/camera_check/camera_info` |
| Image dimensions | 640 × 360 |
| Frame rate, message timestamps | 15.15 Hz |
| Frame rate, wall clock | 13.93 Hz over 9.906 s |
| Pixel contrast | BGR standard deviation 65.70 / 51.56 / 42.60 |
| Probe verdict | `valid: true` |

![Camera check frame](images/camera_check.png)

The frame shows the white stripe and all three boxes with correct colours and
shadows. This is a real rendered frame, not a placeholder image.

**Limit:** the Docker rate is not a VM rate. The native VM result is below.

## VM validation (Lima)

I created a native VM with Lima 2.3.0 using Apple Virtualization (`vz`):

- Ubuntu 24.04.5 LTS arm64
- 8 vCPU, 12 GiB RAM, 40 GiB disk
- ROS 2 Jazzy, Gazebo Sim 8.15.0, Python 3.12.3
- Mesa llvmpipe software rendering, headless only

The public repository was cloned inside the VM. `colcon build --symlink-install`
finished both packages and the 50 tests present at that revision passed.

### VM camera check

| Check | VM result |
|---|---|
| Gazebo starts headlessly | yes |
| Camera publishes through ROS 2 | yes |
| Image dimensions | 640 × 360 |
| Frame rate | 4.53 Hz wall, 4.80 Hz message timestamps |
| Rendering backend | OGRE2, Mesa llvmpipe, software |
| Probe verdict | `valid: true`, but 4.5–5.4 Hz is not a usable control rate |

I do not count this as a passing VM camera gate. A 4-vCPU run gave 5.40 Hz
wall / 5.77 Hz timestamps. Disabling shadows on the static camera checker
raised the rate to 13.46 Hz wall / 13.78 Hz timestamps, so rendering is the
bottleneck. The committed world keeps shadows because the release results use
that world.

### VM release scenarios

Final run with the committed world, 8 vCPU, and the normal controller:

| Scenario | Lap | Progress | p95 error | Max error | Valid | Stops |
|---|---:|---:|---:|---:|---:|---:|
| center | no | 27.304 m | 0.107 m | 0.167 m | 18.0% | 5 |
| left +0.3 m | no | 28.504 m | 0.161 m | 0.300 m | 16.5% | 4 |
| right −0.3 m | no | 47.814 m | 0.242 m | 0.300 m | 25.4% | 4 |

The p95 values are over partial runs, not completed laps, so they are not a
release pass.

**Failure cause:** at 4.5–5.4 Hz, camera intervals occasionally exceed the
0.3 s stale limit. The controller commands zero and the car stops. At several
stop positions perception reports `lane_invalid` and does not resume, so the
lap never completes. This is a rendering-rate and recovery limitation, not a
control-equation error. The Docker environment at 13.9 Hz completes all three
laps.

The raw VM results are in [`results-vm/`](../results-vm/).

## Docker observed results

| Item | Observed value |
|---|---|
| Architecture | aarch64 |
| Ubuntu version | Ubuntu 24.04.5 LTS |
| ROS 2 distribution | Jazzy, `ros-jazzy-ros-base` 0.11.0, `ros-jazzy-ros2cli` 0.32.13 |
| ROS–Gazebo bridge | `ros-jazzy-ros-gz` 1.0.24 |
| Gazebo version | Gazebo Sim 8.15.0 (Harmonic vendor packages) |
| Python version | 3.12.3 |
| Rendering engine | OGRE2, EGL 1.5 surfaceless |
| Rendering backend | Mesa llvmpipe (LLVM 20.1.2), software |
| Camera topic | Gazebo and ROS `/camera_check/image` |
| Image dimensions | 640 × 360 |
| Frame rate | 13.93 Hz wall clock, 15.15 Hz message timestamps |
| GUI | Not tested; the container has no display |

## Build and test results

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

## Failures and revisions

- Initial `colcon build` failed because `catkin_pkg` rejected the maintainer
  email `evangelos@localhost`. I replaced it with the reserved placeholder
  `evangelos.kostakis@example.com` in both `package.xml` files and `setup.py`.
  The build then completed.
- The first camera-check run failed because `set -u` in the runner conflicted
  with `/opt/ros/jazzy/setup.bash`. I disabled nounset while sourcing ROS.
- The first successful probe emitted a NumPy deprecation warning from
  `float(max(std))`. I changed it to `float(std.max())` and reran the probe.

## Rendering issues and limitations

- Rendering uses the CPU-only Mesa llvmpipe driver. The wall-clock rate of
  13.93 Hz is close to the configured 15 Hz, but later tests must use simulation
  time rather than wall time.
- Gazebo logged one benign warning from `Ogre2Camera::SetVisibilityMask` about
  reserved mask bits. It did not affect the frames.
- The Gazebo GUI is not available in this Docker setup. Only headless operation
  is supported here.
- The native VM used Mesa llvmpipe too and was slower than Docker. Its scenario
  runs fail the release target. Do not copy Docker rates into VM results.

## Commands run

```bash
docker build --progress=plain -t auto-drive-sim:jazzy-harmonic -f docker/Dockerfile .
./docker/run.sh bash -lc 'gz sim --version'
./docker/run.sh bash -lc 'colcon build --symlink-install'
./docker/run.sh bash -lc 'tools/run_camera_check.sh'
./docker/run.sh bash -lc 'LIBGL_ALWAYS_SOFTWARE=1 eglinfo'
limactl start --name=ros2-jazzy-vm --cpus 8 --memory 12 --disk 40 --vm-type vz template:ubuntu-24.04
limactl shell ros2-jazzy-vm -- bash -lc 'cd ~/ros2-lane-keeping && tools/run_camera_check.sh'
limactl shell ros2-jazzy-vm -- bash -lc 'cd ~/ros2-lane-keeping && tools/run_scenarios.sh'
```

`ros2 --version` is not accepted by ROS 2 Jazzy; package metadata is used for
the version table instead.
