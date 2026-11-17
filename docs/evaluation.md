# Evaluation

The release scenarios use Gazebo model pose and the generated centerline only
inside the evaluator. The perception and controller nodes never subscribe to
either.

The raw results are in [`results/`](../results/). These results and the
demo images were captured from commit `89de4eb` with a 1.5 m/s target and
lookahead gain 1.2. The documentation commit that follows does not change
runtime code. The runner writes one JSON
result and one lateral-error CSV per scenario, plus
[`results/summary.md`](../results/summary.md).

## Setup

| Item | Value |
|---|---|
| Environment | Docker on macOS arm64 |
| OS | Ubuntu 24.04.5 arm64 |
| ROS 2 | Jazzy |
| Simulator | Gazebo Sim 8.15.0 |
| Track | version 1, 134.248 m centerline |
| Lane width | 3.5 m between marking inner edges |
| Vehicle | 3.8 × 1.6 m, 2.4 m wheelbase |
| Camera | 640 × 360, 90° horizontal FOV, 1.3 m high |
| Controller speed | 1.5 m/s |
| Lookahead gain | 1.2 |
| Speed ramp | 0.6 m/s² |
| Curvature limit | 0.15 1/m |
| Yaw-rate limit | 0.6 rad/s |
| Yaw smoothing | alpha 0.35 at 20 Hz |
| Scenario timeout | 130 simulated seconds |

The scenarios use the same world with the car placed at:

- center: `y = 15.0`
- left: `y = 15.3`
- right: `y = 14.7`

All three start at `x = -10.0` with yaw `0.0`.

The evaluator bridge is
[`src/auto_drive_sim/config/eval_bridge.yaml`](../src/auto_drive_sim/config/eval_bridge.yaml).
It maps `/world/car_world/pose/info` to `/evaluation/model_poses`. The normal
bridge used by perception and control does not include it.

## Metrics

- **Lap completion**: cumulative centerline progress reaches the full
  134.248 m. Progress is unwrapped at the start line, so spinning near the
  start does not count.
- **Lap time**: simulated time from the first 0.5 m of progress to the lap
  finish. The initial stationary wait is excluded.
- **Lateral error**: signed distance from the car origin to the nearest
  centerline sample. The table reports mean absolute, 95th-percentile
  absolute, and maximum absolute error.
- **Boundary crossing**: all four corners of the 3.8 × 1.6 m footprint are
  checked against the ±1.75 m lane edges. A single corner outside counts as a
  crossing.
- **Perception valid**: percentage of `LaneEstimate` messages with
  `valid=true` during the run.
- **Stop events**: command speed transitions from above 0.2 m/s to at most
  0.05 m/s after the car has moved.

The evaluator uses simulated time. A slower renderer changes wall-clock
duration, not the metric timeline.

## Docker results

| Scenario | Lap | Lap time (s) | Mean error (m) | p95 error (m) | Max error (m) | Crossing | Valid (%) | Stops |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 88.34 | 0.147 | 0.231 | 0.231 | no | 100.0 | 0 |
| left +0.3 m | yes | 88.37 | 0.169 | 0.249 | 0.300 | no | 100.0 | 0 |
| right −0.3 m | yes | 88.35 | 0.168 | 0.262 | 0.300 | no | 100.0 | 0 |

**Docker release target: PASS. Docker is the supported release environment.**

- All three laps completed.
- No footprint corner crossed either lane edge.
- All three p95 values are below 0.35 m.

![Lateral error over one lap](images/evaluation_lateral_error.png)

The first offset seconds on the left/right runs show the initial lateral offset
converging back to the lane center. The rest of the lap stays within the
±0.35 m target band.

## Tuning record

I tuned only the controller lookahead gain against the centered scenario. The
scenario definitions and evaluator were not changed between runs.

| Lookahead gain | p95 error (m) | Result |
|---:|---:|---|
| 1.0 | 0.434 | inside offset on both curves |
| 0.8 | 0.634 | worse corner cutting |
| 1.2 | 0.231 | used for the final runs |

A shorter lookahead made the inside offset worse. Increasing the gain to 1.2
gave the car a longer effective lookahead and reduced the curve error without
causing oscillation.

## Speed planning

The controller now limits speed from the lookahead curvature:

```text
v_curve = sqrt(a_lat / |κ|)
```

With `a_lat = 0.15 m/s²`, the 15 m oval curves cap the speed at 1.5 m/s. The
1.5 m/s release target is therefore unchanged. I also ran the solid track with a
2.0 m/s target, so the straights run at 2.0 m/s and the curves at about
1.5 m/s.

| Profile | Center lap | Left lap | Right lap | p95 center / left / right | Stops |
|---|---:|---:|---:|---:|---:|
| 1.5 m/s release | 88.34 s | 88.37 s | 88.35 s | 0.231 / 0.249 / 0.262 m | 0 / 0 / 0 |
| 2.0 m/s planned | 81.00 s | 89.40 s | 87.34 s | 0.231 / 0.293 / 0.232 m | 0 / 7 / 4 |

**2.0 m/s speed planning: PASS.** The center lap is about 8% faster. The offset
runs are within 1–2% of the 1.5 m/s times and have more stale-frame stops. Raw
results are in [`results-fast/`](../results-fast/). The 2.0 m/s profile is
validated on the solid track only.

## Two-lane track

Track version 3 adds a second 3.5 m lane. The car drives in the right lane, and
perception uses `lane_strategy=rightmost` to keep the two rightmost boundary
lines. The right lane centreline is 1.81 m right of the road centre and is
122.887 m long.

| Scenario | Lap | Lap time (s) | Mean error (m) | p95 error (m) | Max error (m) | Crossing | Valid (%) | Stops |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 83.62 | 0.171 | 0.269 | 0.270 | no | 99.5 | 0 |
| left +0.3 m | yes | 83.64 | 0.171 | 0.270 | 0.300 | no | 99.6 | 0 |
| right −0.3 m | yes | 83.62 | 0.171 | 0.270 | 0.300 | no | 99.8 | 0 |

**Two-lane right-lane check: PASS.** The lap times are shorter than the
one-lane times because the right lane is shorter, not because the car is faster.
Raw results are in [`results-two-lane/`](../results-two-lane/).

## Lead-vehicle check

The two-lane lead world places a static red vehicle in the right lane at world
`x = 8 m`. The ego car starts at `x = -10 m`. The lead detector runs from the
camera, and the controller applies the follow-speed cap.

| Check | Result |
|---|---|
| Lead detected by camera | yes |
| Initial detected distance | 8.00 m |
| Stopped detected distance | 5.953 m |
| Final speed | 0.0 m/s |
| Controller state | `lead_vehicle` |
| Stopped at sim time | 11.4 s |
| Collision | no |

**Lead-vehicle stop check: PASS.** The camera estimate is about 6 m at the
stop, and the car holds zero speed. This is a static lead check, not moving
traffic. The result is in [`results-lead/`](../results-lead/).

## Dashed and worn track

Track version 2 keeps the same oval, start poses, controller, and evaluator. It
changes only the markings: 3 m painted dashes, 0.75 m gaps, and every third
dash split by a 0.18 m gap, narrowed to 75% width, and rendered in a dimmer
worn material.

These dashed results and the shadow results below predate the latest steering
and recovery changes. The solid release and the 2.0 m/s profile use the current
controller.

| Scenario | Lap | Lap time (s) | Mean error (m) | p95 error (m) | Max error (m) | Crossing | Valid (%) | Stops |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 91.55 | 0.135 | 0.232 | 0.233 | no | 100.0 | 4 |
| left +0.3 m | yes | 92.52 | 0.150 | 0.245 | 0.300 | no | 100.0 | 5 |
| right −0.3 m | yes | 87.86 | 0.170 | 0.270 | 0.300 | no | 100.0 | 0 |

**Dashed and worn track: PASS.**

The metrics are close to the solid track. Center and left had brief
`stale_lane_estimate` stops from delayed frames; the lap target does not require
zero stops, but the counts are reported. The perception changes are described
in [`perception.md`](perception.md): dashed pieces are paired by forward
distance and side, and a single dash near the 4 m target can use a line fit.
Raw results are in [`results-markings/`](../results-markings/).

## Shadow and low-light check

I added a low-sun world with four shadow-casting buildings. The same six camera
poses used for the dashed frame set were captured under those shadows. All six
frames are valid. The largest lateral-error difference from ground truth is
0.014 m and the largest width difference is 0.057 m.

All three scenarios pass the same target under the shadows:

| Scenario | Lap | Lap time (s) | Mean error (m) | p95 error (m) | Max error (m) | Crossing | Valid (%) | Stops |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| center | yes | 95.56 | 0.123 | 0.230 | 0.238 | no | 96.4 | 8 |
| left +0.3 m | yes | 96.98 | 0.137 | 0.249 | 0.300 | no | 96.4 | 10 |
| right −0.3 m | yes | 95.73 | 0.144 | 0.300 | 0.300 | no | 96.9 | 8 |

**Shadow driving check: PASS in the recorded run.**

This is a stress check, not part of the supported release target. Repeat center
runs stopped at 34.4 m and 113.2 m, so the shadow check is not repeatable yet.

The shadow runs are slower and stop more often than the solid track. Perception
republishes the last valid estimate for up to 0.25 s on a brief invalid frame,
but the estimate keeps its original header timestamp. The controller still
stops once the estimate is older than 0.3 s, and the evaluator counts only fresh
valid messages. I also tried a fallback that drove from one visible boundary
plus the known lane width. It caused a lane departure in one run, so it is not
in the shipped code. Raw results are in
[`results-lighting/`](../results-lighting/).

## VM validation

I also ran the same three scenarios on a native Ubuntu 24.04 ARM64 VM created
with Lima 2.3.0 (Apple Virtualization), 8 vCPU, 12 GiB RAM, and Mesa llvmpipe.
The VM built both packages and passed the 50 tests present at that revision.
It produced valid 640 ×
360 camera frames, but at only 4.53 Hz wall / 4.80 Hz timestamps. That is below
the rate the 0.3 s stale rule needs.

| Scenario | Lap | Progress | p95 error | Max error | Valid | Stops |
|---|---:|---:|---:|---:|---:|---:|
| center | no | 27.304 m | 0.107 m | 0.167 m | 18.0% | 5 |
| left +0.3 m | no | 28.504 m | 0.161 m | 0.300 m | 16.5% | 4 |
| right −0.3 m | no | 47.814 m | 0.242 m | 0.300 m | 25.4% | 4 |

**VM release target: FAIL.**

The camera intervals occasionally exceed the 0.3 s stale limit at 4.5–5.4 Hz.
The controller stops, and at several stop positions perception reports
`lane_invalid` and does not resume. The lap never completes. Disabling shadows
raised the static camera rate to 13.46 Hz, so software rendering is the
bottleneck. These are partial-run p95 values, not completed-lap values. Raw
results are in [`results-vm/`](../results-vm/).

Docker at 13.9 Hz remains the only environment that completes all three laps.

## Run it

```bash
./docker/run.sh bash -lc 'colcon build --symlink-install'
./docker/run.sh bash -lc 'tools/run_scenarios.sh'
cat results/summary.md
```

Run one scenario:

```bash
./docker/run.sh bash -lc 'tools/run_scenarios.sh center'
```

Run the dashed and worn track:

```bash
./docker/run.sh bash -lc 'WORLD=car_world_markings.sdf \
  RESULTS_DIR=results-markings \
  RUN_DIR=artifacts/scenarios-markings \
  tools/run_scenarios.sh'
```

Run the low-sun shadow check:

```bash
./docker/run.sh bash -lc 'WORLD=car_world_lighting.sdf \
  RESULTS_DIR=results-lighting \
  RUN_DIR=artifacts/scenarios-lighting \
  tools/run_scenarios.sh'
```

Run the 2.0 m/s speed-planning profile:

```bash
./docker/run.sh bash -lc 'TARGET_SPEED=2.0 \
  RESULTS_DIR=results-fast \
  RUN_DIR=artifacts/scenarios-fast \
  tools/run_scenarios.sh'
```

The runner writes generated worlds to `artifacts/scenarios/` by default and
committed results to `results/`. The `WORLD`, `RESULTS_DIR`, and `RUN_DIR`
variables select the variant.

## Limitations

- The passing results are from Docker on macOS. The native VM was tested and failed (see above).
- The dashed variant uses 3 m dashes with 0.75 m gaps and a fixed wear pattern.
  Long gaps, heavy wear, and wet markings are not tested.
- Low-sun shadows pass with 8–10 stop events and 96.4–96.9% fresh-valid
  perception. A longer perception loss still stops the car.
- Driving from one visible boundary was tested and rejected; the car can leave
  the lane when the wrong edge is used.
- There is no traffic, weather, or occlusion.
- The lateral error is measured to the nearest centerline sample; the footprint
  crossing check uses the same centerline.
- Meeting this target on this track is not a claim of road-ready autonomy.
