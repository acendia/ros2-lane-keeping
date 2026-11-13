# Car

The first-release car is an original SDF model built from boxes and cylinders.
It uses Gazebo Harmonic's Ackermann steering system. It is not a model copied
from another project.

## Geometry and mass

| Item | Value |
|---|---|
| Body length | 3.8 m |
| Body width | 1.6 m |
| Body height | 0.5 m |
| Chassis mass | 700 kg |
| Wheelbase | 2.4 m |
| Wheel separation | 1.4 m |
| Wheel radius | 0.32 m |
| Wheel width | 0.2 m |
| Wheel mass | 12 kg each |
| Steering link mass | 1 kg each |
| Steering limit | ±0.55 rad |
| Tyre friction | μ = 1.0, μ₂ = 0.8 |
| Drive | Rear-wheel drive |
| Steering | Front-wheel Ackermann |

The model origin is at ground level between the wheels. Positive X is forward,
positive Y is left, and positive Z is up. The car is spawned at the track start
`(-10.0, 15.0, 0.05)` with yaw `0.0`, facing east along the top straight.

## Joints and plugin

Each front wheel has a steering link. The steering joints rotate about the
vertical (Z) axis with limits of ±0.55 rad. Each wheel spins about its axle.
The rear wheel joints are driven by:

```xml
<plugin filename="gz-sim-ackermann-steering-system"
        name="gz::sim::systems::AckermannSteering">
  <left_joint>rear_left_wheel_joint</left_joint>
  <right_joint>rear_right_wheel_joint</right_joint>
  <left_steering_joint>front_left_wheel_steering_joint</left_steering_joint>
  <right_steering_joint>front_right_wheel_steering_joint</right_steering_joint>
  <kingpin_width>1.4</kingpin_width>
  <steering_limit>0.55</steering_limit>
  <wheel_base>2.4</wheel_base>
  <wheel_separation>1.4</wheel_separation>
  <wheel_radius>0.32</wheel_radius>
  <min_velocity>-1.0</min_velocity>
  <max_velocity>3.0</max_velocity>
  <min_acceleration>-1.5</min_acceleration>
  <max_acceleration>1.5</max_acceleration>
  <odom_publish_frequency>20</odom_publish_frequency>
  <topic>/car/cmd_vel</topic>
  <odom_topic>/car/odometry</odom_topic>
  <tf_topic>/car/odom_tf</tf_topic>
</plugin>
```

## Camera

The forward RGB camera is mounted on a mast on the chassis.

| Item | Value |
|---|---|
| Position | 1.4 m forward, 1.3 m above ground |
| Pitch | 0.05 rad down |
| Resolution | 640 × 360 |
| Rate | 15 Hz |
| Horizontal field of view | 60° |
| Gazebo topic | `/car/camera/image_raw` |
| ROS topic | `/car/camera/image_raw` |

![Car camera at the track start](images/car_start_camera.png)

The camera image shows the lane ahead with both boundary lines and no part of
the car body in view.

## ROS interface

The bridge mappings are in
[`src/auto_drive_sim/config/bridge.yaml`](../src/auto_drive_sim/config/bridge.yaml).
This file is the single source of truth for topic names and directions.

| ROS topic | Type | Direction |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS |
| `/car/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS |
| `/car/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS |
| `/car/odometry` | `nav_msgs/msg/Odometry` | Gazebo → ROS |
| `/car/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo |

The Ackermann odometry is local to the spawn pose: the first message reports
position `(0, 0, 0)` and identity orientation. Gazebo model pose remains the
ground-truth source for the evaluator and is not used by perception or control.

## Manual driving

Start the world, bridge, and optional teleop node:

```bash
ros2 launch auto_drive_sim manual_drive.launch.py gui:=false teleop:=true
```

The teleop node is interactive and needs a terminal. Without one, run it in a
second terminal after starting the launch without `teleop:=true`:

```bash
ros2 launch auto_drive_sim manual_drive.launch.py gui:=false
ros2 run auto_drive_sim teleop
```

Keys: `w` forward, `s` reverse, `a` left, `d` right, space stop, `q` quit.

A fixed command can be sent without a keyboard:

```bash
ros2 run auto_drive_sim teleop --linear 0.5 --angular 0.0 --duration 2.0
```

`linear` is metres per second and `angular` is the commanded yaw rate in radians
per second. The node sends a stop command before it exits.

## Step 6 checks

These checks verify the model, spawn, and topic plumbing. They do not yet claim
the full manual-driving gate.

| Check | Result |
|---|---|
| Car SDF validates | Yes |
| Car world validates | Yes |
| Car spawns at the start | Yes, ground truth `(-10.0, 15.0, -0.02)` after settling |
| `/clock` present | Yes |
| `/car/camera/image_raw` present | Yes |
| `/car/camera/camera_info` present | Yes |
| `/car/odometry` present | Yes |
| `/car/cmd_vel` present | Yes |
| Camera stream | 49 frames in 4 s, 640 × 360, 12.22 Hz wall, 14.84 Hz timestamps |
| Forward command path | 0.5 m/s for 2 s moved odometry 0.6355 m |
| Ground-truth motion | x changed from −10.0 to −9.3645 at y = 15.0 |
| Camera orientation | Start straight visible with both boundaries |

## Pending driving verification

Still to verify in the next step:

- left and right steering signs,
- stopping after a command ends,
- a manual pass along a straight and through a curve,
- wheel rotation direction at the joint level.

The car must not be used for autonomous control until those checks pass.
