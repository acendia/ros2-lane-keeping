# Scenario results

Environment: Lima VM, Ubuntu 24.04.5 arm64, 8 vCPU, 12 GiB, ROS 2 Jazzy, Gazebo Sim 8.15.0, Mesa llvmpipe.

| Scenario | Lap | Lap time (s) | p95 error (m) | Max error (m) | Boundary crossing | Perception valid (%) | Stop events |
|---|---:|---:|---:|---:|---:|---:|---:|
| center | no | — | 0.107 | 0.167 | no | 18.0 | 5 |
| left | no | — | 0.161 | 0.300 | no | 16.5 | 4 |
| right | no | — | 0.242 | 0.300 | no | 25.4 | 4 |

Release target: **FAIL**

- All three laps complete: no
- No footprint boundary crossing: yes
- p95 absolute lateral error below 0.35 m: yes
