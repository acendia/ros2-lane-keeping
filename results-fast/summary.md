# Scenario results

Environment: Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0. Speed planning: 2.0 m/s straights, 1.5 m/s curves.

| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | Boundary crossing | Perception valid (%) | Stop events |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 81.00 | 0.231 | 0.232 | no | 100.0 | 0 |
| left | yes | 84.44 | 0.300 | 0.300 | no | 100.0 | 4 |
| right | yes | 82.66 | 0.281 | 0.300 | no | 100.0 | 1 |

Release target: **PASS**

- All three laps complete: yes
- No footprint boundary crossing: yes
- p95 absolute lateral error below 0.35 m: yes
