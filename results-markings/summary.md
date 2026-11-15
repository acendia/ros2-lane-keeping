# Scenario results

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0. Track version 2, dashed boundaries with worn segments.

| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | Boundary crossing | Perception valid (%) | Stop events |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 91.55 | 0.232 | 0.233 | no | 100.0 | 4 |
| left | yes | 92.52 | 0.245 | 0.300 | no | 100.0 | 5 |
| right | yes | 87.86 | 0.270 | 0.300 | no | 100.0 | 0 |

Release target: **PASS**

- All three laps complete: yes
- No footprint boundary crossing: yes
- p95 absolute lateral error below 0.35 m: yes
