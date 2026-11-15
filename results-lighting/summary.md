# Scenario results

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0. Track version 2, dashed and worn boundaries, low sun with building shadows.

| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | Boundary crossing | Perception valid (%) | Stop events |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | no | — | 0.001 | 0.001 | no | 11.6 | 2 |
| left | no | — | 0.207 | 0.300 | no | 74.9 | 15 |
| right | no | — | 0.118 | 0.300 | no | 8.9 | 1 |

Release target: **FAIL**

- All three laps complete: no
- No footprint boundary crossing: yes
- p95 absolute lateral error below 0.35 m: yes
