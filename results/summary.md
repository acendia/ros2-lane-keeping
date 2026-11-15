# Scenario results

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0.

| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | Boundary crossing | Perception valid (%) | Stop events |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 87.87 | 0.231 | 0.232 | no | 100.0 | 0 |
| left | yes | 87.90 | 0.255 | 0.300 | no | 100.0 | 0 |
| right | yes | 87.86 | 0.276 | 0.300 | no | 100.0 | 0 |

Release target: **PASS**

- All three laps complete: yes
- No footprint boundary crossing: yes
- p95 absolute lateral error below 0.35 m: yes
