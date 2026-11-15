# Scenario results

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0. Track version 2, dashed boundaries with worn segments.

| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | Boundary crossing | Perception valid (%) | Stop events |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 87.87 | 0.233 | 0.233 | no | 100.0 | 0 |
| left | yes | 87.89 | 0.252 | 0.300 | no | 100.0 | 0 |
| right | yes | 87.86 | 0.264 | 0.300 | no | 100.0 | 0 |

Release target: **PASS**

- All three laps complete: yes
- No footprint boundary crossing: yes
- p95 absolute lateral error below 0.35 m: yes
