# Lead-vehicle check

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0.
Two-lane track, right lane, static lead vehicle at world `x = 8 m`.

| Check | Result |
|---|---|
| Lead detected by camera | yes |
| Initial detected distance | 8.00 m |
| Stopped detected distance | 5.953 m |
| Final speed | 0.0 m/s |
| Controller state | `lead_vehicle` |
| Stopped at sim time | 11.4 s |
| Collision | no |

The ego car approached the lead from `x = -10 m` and stopped with about 6 m
of camera-to-lead distance. The check passes.
