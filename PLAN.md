# Camera-Based Lane Following in Gazebo

## Goal

Build an original autonomous-driving simulation in which a car uses its forward camera to detect two painted lane boundaries and complete a closed loop without leaving the lane. The first release should show a working simulation, understandable Python code, repeatable measurements, and a clear GitHub presentation.

This is a simulation project, not a claim of road-ready autonomy. The first release covers lane recognition and lane keeping on one marked lane. It does not cover traffic, obstacles, intersections, or lane changes.

## Chosen stack and design

| Area | First-release choice | Why |
|---|---|---|
| Runtime | Ubuntu 24.04 ARM64 VM on Apple Silicon | Gives the project a Linux ROS environment while keeping development on the Mac. |
| Middleware | ROS 2 Jazzy | Provides Python nodes, standard message types, launch files, and an architecture familiar to robotics developers. |
| Simulator | Gazebo Harmonic | This is the recommended Gazebo pairing for ROS 2 Jazzy. |
| Vehicle | Original four-wheel SDF car using Gazebo's Ackermann steering system | Gives car-like steering without writing a simulator plugin in the first release. |
| Road | Original generated oval with one 3.5 m lane and continuous white boundaries | Supports repeatable laps and clear measurements. |
| Perception | Python and OpenCV, using classical computer vision | Makes the lane-recognition logic inspectable and gives a baseline for future approaches. |
| Control | Python pure-pursuit lane follower | Uses the center of the detected lane to produce steering commands. |
| Evaluation | Separate ground-truth process | Measures actual behavior without giving the controller privileged simulator data. |

Gazebo and ROS recommend the [Jazzy–Harmonic pairing](https://gazebosim.org/docs/harmonic/ros_installation/). ROS 2 Jazzy supports Ubuntu 24.04 ARM64, while Gazebo Harmonic lists ARM as best-effort support. The first milestone must therefore prove camera rendering in the VM before later work depends on it. See [ROS Jazzy platform support](https://discourse.ros.org/t/ros-2-jazzy-jalisco-released/37862) and [Gazebo Harmonic platform support](https://gazebosim.org/docs/harmonic/install/).

### Alternatives considered

- **Native macOS:** Gazebo has a macOS install path, but its GUI is documented as unstable, and the current Mac version is outside the documented prebuilt-binary target. Use the Ubuntu VM as the primary supported environment. See the [Gazebo macOS notes](https://gazebosim.org/docs/harmonic/).
- **Direct Gazebo transport without ROS:** This would reduce initial setup, but ROS 2 makes the Python perception, control, and evaluation components easier to separate and present. Gazebo and ROS communicate through `ros_gz`. See the [integration guide](https://gazebosim.org/docs/harmonic/ros2_integration/).
- **Learned lane detection:** Keep it for a later comparison. A classical detector is easier to explain, test, and debug on the deliberately simple first track.
- **Custom vehicle physics:** Keep it for later. Gazebo's [Ackermann steering system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html) is enough to establish the vision-to-control loop.

## First-release operating assumptions

- Flat road, daylight, one car, no other road users, and no obstacles.
- Two continuous, high-contrast white boundaries mark one lane. There is no center dividing line.
- Oval track: two 20 m straights joined by 15 m radius ends; lane width 3.5 m.
- Simple car: approximately 3.8 m long, 1.6 m wide, with a 2.4 m wheelbase.
- Fixed forward camera at approximately 1.3 m height, initially 640 × 360 at 15 Hz.
- Initial target speed: 1.5 m/s. These parameters belong in configuration and may be tuned from measured runs.
- Camera perception determines steering. Odometry may regulate speed. The controller may not read the track centerline or simulator pose.
- A missing or stale lane estimate makes the car stop. Camera frames older than 0.3 s count as stale.
- Headless Gazebo with a working camera is required. The Gazebo GUI is useful for inspection and recording but is not required for correctness.

## Repository shape

Use a ROS workspace with two packages:

```text
auto-drive-sim/
├── PLAN.md
├── README.md
├── LICENSE
├── .gitignore
├── src/
│   ├── auto_drive_sim/                 # ament_python package
│   │   ├── auto_drive_sim/             # Python nodes and pure algorithms
│   │   ├── config/                     # camera, perception, and control settings
│   │   ├── launch/                     # one-command demo and test launches
│   │   ├── models/                     # original car and generated road assets
│   │   └── worlds/                     # Gazebo world
│   └── auto_drive_sim_interfaces/      # LaneEstimate.msg
├── tools/                              # original track generator and scenario tools
├── tests/                              # geometry, perception, control, integration
└── docs/                               # setup, architecture, experiments, results
```

Commit the generated road assets alongside their generator. A fresh clone should launch without downloading models or generating meshes first. Keep the generator so the geometry is inspectable and reproducible.

The intended build and launch interface is:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch auto_drive_sim demo.launch.py gui:=false
```

`gui:=true` starts the Gazebo window when the VM supports it. The launch file starts the world, bridge, perception node, controller, and optional evaluator with `use_sim_time` enabled.

### ROS interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/car/camera/image_raw` | `sensor_msgs/msg/Image` | Forward camera input. |
| `/car/odometry` | `nav_msgs/msg/Odometry` | Speed feedback. |
| `/car/lane_estimate` | `auto_drive_sim_interfaces/msg/LaneEstimate` | Timestamp, validity, confidence, lateral error, and lookahead target in vehicle coordinates. |
| `/car/cmd_vel` | `geometry_msgs/msg/Twist` | Speed and yaw-rate command bridged to Gazebo's Ackermann system. |
| `/car/debug_image` | `sensor_msgs/msg/Image` | Optional boundary-fit and target overlay. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time. |

Bridge Gazebo model pose separately for the evaluator. Neither the perception nor controller node may subscribe to it. Put bridge mappings and directions in one checked-in configuration file. The [ROS–Gazebo bridge](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md) supports the message types needed for this design.

## How to use the implementation prompts

Feed **one milestone at a time** to an LLM. Before each milestone, provide the current repository and prepend this standing instruction:

> Work only on the milestone below. Inspect the current repository first and preserve completed milestones. Implement the requested code and documentation, run the listed checks, and report files changed, commands run, observed results, and unresolved issues. Do not copy perception, control, vehicle, or road logic from another repository. Dependencies such as Gazebo, ROS 2, and OpenCV are allowed. Keep public writing direct, factual, and in first person singular. Do not claim a test passed unless you ran it.

Each milestone ends at an observable gate. Resolve failures before moving to the next one.

### Milestone 0 — Validate the VM and establish the project

**Prompt**

> Set up this empty repository for Ubuntu 24.04 ARM64, ROS 2 Jazzy, Gazebo Harmonic, and Python. Document exact installation and verification commands from official ROS and Gazebo instructions. Create the two-package ROS workspace, dependency declarations, `.gitignore`, MIT license, and a minimal README. First prove that Gazebo starts headlessly and publishes actual frames from a test camera at a usable rate. Record the VM architecture, installed versions, camera topic, image dimensions, observed frame rate, and any rendering issues in `docs/environment.md`. Do not proceed to the custom road until this camera check passes. If the GUI fails but headless camera rendering works, continue with headless as the supported path and document the GUI limitation. If camera rendering fails, stop and report the exact error and environment details.

**Gate:** A fresh VM builds the workspace and receives camera frames through ROS. This is a feasibility check, not a placeholder “hello world” completion.

### Milestone 1 — Create the road and world

**Prompt**

> Write an original Python track generator that produces an oval centerline, asphalt mesh, and two continuous white boundary meshes. Use two 20 m straights, 15 m radius ends, and a 3.5 m lane width. Store the geometry parameters in one configuration source and export centerline samples for evaluation. Build a Gazebo world with a flat ground plane, simple lighting, the road, and a defined start pose. Include the generator and its generated assets in the repository. Verify the lane markings are visible in camera images and do not flicker or disappear at mesh seams. Document how to regenerate the assets and what coordinate frame the track uses.

**Gate:** Gazebo loads the world without missing assets; a camera image shows both continuous boundary lines on straights and curves; regeneration produces the same geometry.

### Milestone 2 — Build and manually drive the car

**Prompt**

> Create an original SDF car from simple geometry with a chassis, four wheels, physical joints, collision shapes, a forward RGB camera, and Gazebo's Ackermann steering system. Set wheelbase, wheel radius, steering limit, mass, and friction to explicit documented values. Place the car at the track start, aligned with the lane tangent. Add the ROS–Gazebo bridge for camera, odometry, `/clock`, and velocity commands. Provide a small manual-drive command or Python teleoperation utility and a launch option that disables autonomy. Verify forward motion, left and right turns, stopping, camera orientation, and topic names. Correct any wheel rotation or steering-sign errors before adding autonomous control.

**Gate:** The car can be driven around a straight and a curve manually. Camera and odometry topics contain sensible data.

### Milestone 3 — Implement lane perception

**Prompt**

> Write a Python ROS node that receives only the forward camera image for lane geometry. Calibrate a fixed ground-plane view from the camera pose and intrinsics; crop to useful road pixels; segment the white markings; fit separate left and right boundaries; estimate the lane center and a target point approximately 4 m ahead. Publish `LaneEstimate` and a debug overlay. Keep image processing in pure functions that can be tested with saved frames. Require two plausible boundaries for the first release. Reject fits that are stale, swapped, implausibly narrow or wide, or too sparse; publish `valid=false` instead of inventing a lane. Test saved straight, left-curve, right-curve, and no-line frames. Explain each processing stage with one annotated image in `docs/perception.md`.

**Gate:** Saved test frames produce the expected center and boundary fits. Blank or misleading frames produce an invalid estimate.

### Milestone 4 — Implement the lane follower

**Prompt**

> Write a separate Python controller node that consumes `LaneEstimate` and odometry. Use pure pursuit to turn the camera-derived lookahead point into curvature, then publish bounded speed and yaw-rate commands compatible with the Ackermann system. Start at 1.5 m/s; make speed, lookahead, steering limits, and smoothing configurable. Stop the car when the lane estimate is invalid, the image timestamp is older than 0.3 seconds, or odometry is stale. On restart or recovery, ramp speed back up rather than jumping to the target. The controller must never read the generated track path or simulator pose. Add unit tests for straight, left, right, excessive-curvature, stale-data, and lost-lane cases.

**Gate:** The car follows the straight and both curve directions from camera input. Loss of perception leads to a stop.

### Milestone 5 — Add repeatable evaluation and tune

**Prompt**

> Build a scenario runner and evaluator. Use Gazebo model pose and the generated track centerline only in this evaluation process. Run one lap from centered, 0.3 m left, and 0.3 m right starting positions. Compute lap completion, lateral error over simulated time, 95th-percentile absolute lateral error, lane-boundary crossings using the vehicle footprint, perception-valid percentage, and stop events. Save machine-readable JSON and a concise Markdown result table. Tune the existing perception and control parameters against these scenarios without hiding failures or changing the test definitions. Keep test duration in simulation time so slower VM rendering does not change the result.

**Release target:** All three scenarios complete a lap, no part of the car crosses a lane boundary, and 95th-percentile absolute lateral error stays below 0.35 m. If a target is missed, publish the measured result and failure cause instead of claiming completion.

### Milestone 6 — Make the GitHub release understandable

**Prompt**

> Finish public documentation and demo assets. Open the README with one sentence describing the actual capability and a short clip or screenshot sequence showing the camera view, detected lines, and car following the road. Include a results table with the exact tested environment, then architecture, prerequisites, copyable setup and launch commands, test commands, repository layout, known limitations, and the next milestone. Add an architecture diagram and a short explanation of the perception and control equations in `docs/`. Capture a reproducible demo run and state which commit and configuration produced it. Check that a new reader can follow setup from a clean Ubuntu VM without private files, network-fetched models, or undocumented manual steps.

**Gate:** Another developer can build, run, understand, and reproduce the reported lap results from the public repository.

## Verification and release rules

- Run fast unit tests for geometry generation, lane fitting, and control math on every relevant change. Run Gazebo integration scenarios after milestones 2, 4, and 5.
- Separate **observed results** from **targets**. Record Ubuntu, architecture, ROS, Gazebo, camera settings, track version, configuration, and commands used for every published result.
- A lane departure means any vehicle footprint corner crosses either boundary. A completed lap requires crossing the start line after progressing around the full oval; spinning near the start does not count.
- Keep simulator ground truth out of the perception and controller processes. Its only role is evaluation.
- Treat a missing or stale lane estimate as a stop condition. Recovery, dashed markings, occlusion, adverse weather, and obstacle avoidance belong to later milestones.
- A test or demo failing on the ARM VM must be documented with the exact failure. Do not silently switch ROS or Gazebo versions, change the architecture, or claim a result measured elsewhere.

## Roadmap after the first release

1. **Perception robustness:** Add dashed and worn markings, partial visibility, shadows, and lighting changes. Build a labeled frame set and measure detection quality before changing algorithms.
2. **Driving quality:** Add speed planning through curves, smoother steering, and recovery after brief perception loss. Compare against first-release metrics on the same track.
3. **Road complexity:** Add multiple lanes, intersections, lane choice, and a route goal. Introduce planning only when single-lane behavior is stable.
4. **Other road users:** Add obstacle detection, following distance, stopping, and basic right-of-way scenarios, each with measurable tests.
5. **Broader realism:** Add sensor noise, varied vehicle dynamics, localization error, and more tracks. Compare a learned lane detector with the classical baseline only when a dataset and evaluation protocol exist.

## Public writing instructions

Write as an engineer documenting work actually done: **“I built…,” “I measured…,” “This version assumes…,” “The car stops when…”** Use short sentences, exact commands, figures, and screenshots. Explain failures and revisions in engineering notes. Avoid claims such as “fully autonomous,” “production ready,” or “robust in all conditions.” Keep the README focused on running and judging the project; put derivations and experiments in `docs/`. The repository should make the original perception and control work easy to find and the limits easy to understand.
