# Scenario results

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0. Track version 2, dashed and worn boundaries, low sun with building shadows.

| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | Boundary crossing | Perception valid (%) | Stop events |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 95.56 | 0.230 | 0.238 | no | 96.4 | 8 |
| left | yes | 96.98 | 0.249 | 0.300 | no | 96.4 | 10 |
| right | yes | 95.73 | 0.300 | 0.300 | no | 96.9 | 8 |

Release target: **PASS**

- All three laps complete: yes
- No footprint boundary crossing: yes
- p95 absolute lateral error below 0.35 m: yes
