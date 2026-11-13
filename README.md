# ros2-lane-keeping

A camera-based lane keeping project for ROS 2 Jazzy and Gazebo Harmonic.

I'm building a small car that follows a painted lane using only its forward
camera. The simulator provides the world and the car. The camera gives the lane
geometry.

![Car camera at the start of the track](docs/images/car_start_camera.png)

## Status

Milestone 3 is complete in Docker. The forward camera now produces a lane
estimate on straights and both curve directions, and rejects blank or no-line
frames. The next step is the pure-pursuit controller. The planned Ubuntu
24.04 ARM64 VM is still untested.

What I have measured:

- Camera: 640 × 360, 12.22 Hz wall clock, 14.84 Hz message timestamps.
- Manual straight + curve: 39.65 m, heading change −1.31 rad, max lane error
  0.188 m.
- Perception: straight error 0.004 m, right curve +1.066 m, left curve
  −1.061 m.
- Live perception: confidence 0.873, error 0.0036 m at the start pose.
- Track: 134.25 m centerline, 3.5 m lane between the markings.
- Tests: 28 passed.

## Repository layout

```text
ros2-lane-keeping/
├── src/
│   ├── auto_drive_sim/                 # Python nodes, config, launch, models, worlds
│   └── auto_drive_sim_interfaces/      # LaneEstimate.msg
├── tools/                              # Track generator and driving tools
├── tests/                              # Geometry, car, and bridge tests
├── docker/                             # Ubuntu 24.04 + ROS 2 Jazzy image
└── docs/                               # Environment, track, and car notes
```

The ROS packages still use the `auto_drive_sim` name. I kept them to avoid a
risky rename. The project name is ros2-lane-keeping.

## Run it

Docker is the current development setup:

```bash
./docker/run.sh bash -lc 'colcon build --symlink-install'
./docker/run.sh bash -lc 'tools/run_road_camera_check.sh'
```

The camera check starts Gazebo headlessly and writes four views to
`artifacts/road_check/`.

Manual drive:

```bash
./docker/run.sh
```

Inside the container:

```bash
source install/setup.bash
ros2 launch auto_drive_sim manual_drive.launch.py gui:=false &
sleep 18
ros2 run auto_drive_sim teleop --linear 0.5 --angular 0.0 --duration 2.0
```

For keyboard control, run `ros2 run auto_drive_sim teleop` while the launch is
running. Keys: `w/a/s/d`, space to stop, `q` to quit.

A recorded straight-and-curve run uses the segment option:

```bash
ros2 run auto_drive_sim teleop \
  --segments "1.5,0,14.9;1.5,-0.1,16.0" \
  --tail 2.0 \
  --record /tmp/drive.csv
```

## Run lane perception

```bash
ros2 launch auto_drive_sim perception.launch.py gui:=false
ros2 topic echo /car/lane_estimate
```

The debug overlay is on `/car/debug_image`. Saved test frames and the full
pipeline are documented in [`docs/perception.md`](docs/perception.md).

On a native Ubuntu 24.04 machine with ROS 2 Jazzy and Gazebo Harmonic:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch auto_drive_sim track_preview.launch.py gui:=false
```

`gui:=true` needs a display. It is not tested in Docker.

## Track

- Two 20 m straights and two 15 m radius ends.
- 3.5 m lane between the inner edges of the white markings.
- Start at `(-10, 15)`, heading east along the top straight.

The track is generated from one config file:

```bash
python3 tools/generate_track.py
```

Details are in [`docs/track.md`](docs/track.md).

## Car

- 3.8 × 1.6 m body, 2.4 m wheelbase.
- Rear-wheel drive with Ackermann steering.
- 640 × 360 camera at 1.3 m height.
- Topics: `/car/camera/image_raw`, `/car/odometry`, `/car/cmd_vel`.

Details are in [`docs/car.md`](docs/car.md).

## Tests

```bash
./docker/run.sh bash -lc 'python3 -m pytest -q tests'
```

## Known limitations

- No autonomous controller yet. The car still drives from manual commands.
- Lane perception assumes a flat road and a fixed camera pitch.
- Only continuous white markings are handled.
- Docker renders with Mesa llvmpipe. Four cameras run at 6–10 Hz.
- One lane, one car, no traffic or obstacles.

## License

MIT. See [`LICENSE`](LICENSE).
