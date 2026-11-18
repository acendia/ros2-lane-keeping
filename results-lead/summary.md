# Lead-vehicle checks

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0.
Two-lane track, right lane.

## Static lead stop

Static lead vehicle at world `x = 8 m`.

| Check | Result |
|---|---|
| Lead detected by camera | yes |
| Initial detected distance | 8.00 m |
| Stopped detected distance | 5.978 m |
| Final speed | 0.0 m/s |
| Controller state | `lead_vehicle` |
| Checked for | 25 s |
| Collision | no |

**Static lead stop: PASS.**

## Moving lead follow

Lead car drives the right lane at 1.0 m/s. The ego starts 15 m behind and
follows.

| Check | Result |
|---|---|
| Lead speed | 1.0 m/s |
| Sampled gap range | 7.12–12.96 m |
| Final gap | 8.27 m |
| Final ego speed | 1.44 m/s |
| Collision | no |
| Lead reached the bottom straight | yes |

The gap was sampled every 5 s from the live probe. **Moving lead follow:
PASS.** The lead uses ground-truth centerline driving; the ego uses only the
camera lead detector and lane perception.
