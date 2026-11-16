# Scenario results

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0. Two-lane track, right lane.

| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | Boundary crossing | Perception valid (%) | Stop events |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 83.62 | 0.269 | 0.270 | no | 99.5 | 0 |
| left | yes | 83.64 | 0.270 | 0.300 | no | 99.6 | 0 |
| right | yes | 83.62 | 0.270 | 0.300 | no | 99.8 | 0 |

Release target: **PASS**

- All three laps complete: yes
- No footprint boundary crossing: yes
- p95 absolute lateral error below 0.35 m: yes
