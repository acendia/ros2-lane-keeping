# Environment

**Status: NOT VERIFIED.** No command in this document has been run on the target
environment yet. Nothing here should be read as a passing result.

## Target environment

| Item | Planned value |
|---|---|
| OS | Ubuntu 24.04 ARM64 |
| ROS 2 | Jazzy |
| Simulator | Gazebo Harmonic |
| Python | 3.12 (Ubuntu 24.04 system Python) |
| Host | macOS on Apple Silicon |

## Working environment decision

The plan calls for an Ubuntu 24.04 ARM64 VM. During implementation I chose to
run the Linux environment in Docker on the macOS host instead, because no VM
image was available on the machine. This is a deviation from the plan. Results
measured in Docker must be labelled as such and must not be presented as VM
results.

If the Docker environment cannot render camera frames, I will stop at the
Milestone 0 gate and record the exact error rather than proceeding to the road.

## Installation commands

Not yet recorded. I will add the exact commands from the official ROS 2 Jazzy
and Gazebo Harmonic installation instructions after running them.

## Camera feasibility check

Not yet run. The check must demonstrate:

- Gazebo starts headlessly.
- A test camera publishes actual image frames through ROS 2.
- The image dimensions and frame rate are usable.

## Observed results

| Item | Observed value |
|---|---|
| Architecture | not measured |
| Ubuntu version | not measured |
| ROS 2 distribution and version | not measured |
| Gazebo version | not measured |
| Camera topic | not measured |
| Image dimensions | not measured |
| Frame rate | not measured |
| Rendering backend | not measured |

## Rendering issues and limitations

None recorded yet.

## Commands run

None yet.
