# Setup

Docker is the supported release environment. A native Ubuntu 24.04 ARM64 VM
was also built with Lima; it builds, passes the tests, and produces camera
frames, but the rate is too low and the full scenarios fail. The VM is not
supported for release runs. See [`environment.md`](environment.md). The same
Ubuntu packages apply in both.

## Docker

From the repository root:

```bash
./docker/run.sh
```

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q tests
ros2 launch auto_drive_sim demo.launch.py gui:=false
```

`docker/run.sh` builds the image on first use. The image is Ubuntu 24.04 arm64
with ROS 2 Jazzy, Gazebo Harmonic, OpenCV, and Mesa software rendering.

## Ubuntu 24.04 ARM64

These are the packages used by the Docker image. Install the ROS 2 apt source
from the official `ros2-apt-source` package, then install the ROS and Gazebo
vendor packages:

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common \
  build-essential cmake git python3 python3-pip \
  python3-opencv python3-numpy python3-yaml python3-pytest

sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=1.3.0
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-tools-vendor \
  ros-jazzy-gz-sim-vendor \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-image-view \
  ros-jazzy-foxglove-bridge \
  python3-colcon-common-extensions
```

Do not install `gz-harmonic` from `packages.osrfoundation.org` on top of the
ROS vendor packages. The supported Jazzy pairing is
`ros-jazzy-ros-gz`.

For headless rendering without a GPU, install Mesa:

```bash
sudo apt install -y libgl1-mesa-dri libegl1 libgles2 libglu1-mesa mesa-utils
```

## Lima VM (not supported for release)

I tested a native VM with Lima 2.3.0 and Apple Virtualization:

```bash
limactl start --name=ros2-jazzy-vm --cpus 8 --memory 12 --disk 40 \
  --vm-type vz template:ubuntu-24.04
limactl shell ros2-jazzy-vm
```

Inside that VM, run the Ubuntu package commands above, clone the repository,
build, and run the camera check:

```bash
git clone https://github.com/acendia/ros2-lane-keeping.git
cd ros2-lane-keeping
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q tests
tools/run_camera_check.sh
```

Headless only; there is no Gazebo GUI. The VM produces valid camera frames,
but runs at only 4.5–5.4 Hz. The scenario run fails the release target because
llvmpipe cannot sustain the camera rate. `results-vm/` contains the measured
failure.

## Build

```bash
git clone https://github.com/acendia/ros2-lane-keeping.git
cd ros2-lane-keeping
colcon build --symlink-install
source install/setup.bash
```

The road meshes, car model, and world are committed. Nothing is downloaded from
Gazebo Fuel. `PLAN.md` and `PROGRESS.md` are local working files and are not
part of the public repository.

`ros-jazzy-foxglove-bridge` is the optional live interface. The Docker runner
publishes port 8765 for it.

## Run

Headless:

```bash
ros2 launch auto_drive_sim demo.launch.py gui:=false
```

This starts the world, bridge, perception node, and pure-pursuit controller.
The car starts driving by itself. `Ctrl-C` stops the launch.

`gui:=true` needs a display. It is not tested in Docker.

Manual drive:

```bash
ros2 launch auto_drive_sim manual_drive.launch.py gui:=false
ros2 run auto_drive_sim teleop
```

Perception only:

```bash
ros2 launch auto_drive_sim perception.launch.py gui:=false
ros2 topic echo /car/lane_estimate
```

## Tests

```bash
python3 -m pytest -q tests
```

Scenario evaluation:

```bash
tools/run_scenarios.sh
cat results/summary.md
```

The scenario runner starts one world at a time and writes JSON and CSV results.
It takes several minutes because each run is one simulated lap.
