# Maintenance validation — 2026-11-22

I corrected evaluation failure handling, replaced the pose-array index with an
ego-only pose topic, and introduced stamped lead estimates. Lead mode now stops
when its required stream is missing or expired, and its parameter overrides
reach the controller. The baseline launch keeps lead mode disabled.

## Checks

- ROS packages rebuilt successfully in the existing Docker image.
- 88 tests passed, including parameter overrides, lead-stream loss and recovery,
  camera timestamps, evaluator replay rejection, and failed-run isolation.
- All three baseline scenarios completed with no footprint crossing and no stops.
- Lap times: 88.35 s centered, 88.40 s left, 88.35 s right.
- p95 absolute errors: 0.2306 m, 0.2598 m, and 0.2754 m respectively.
- Full-launch static-lead check: 25.013 simulated seconds, 5.978 m detected
  gap, zero commanded speed. See `results-review/lead_static.json`.
- Git diff whitespace checks and shell syntax checks passed.

The scenario JSON and CSV files are in [`results-review/`](../results-review/).
The companion metadata records the source fingerprint at run start, base commit,
dirty status, installed packages, and requested profile. These are working-tree
measurements, not a tagged release. A later edit added tests to the fingerprint
input and removed an unused evaluator attribute; the run-start fingerprint is
preserved rather than replaced with a fingerprint of later files.

All evaluator processes exited successfully. The runner's trailing summary step
encountered a shell error because I edited its source while Bash was executing
it. I generated the summary directly from the completed, fresh scenario files.
The full runner invocation therefore did not exit successfully. The final script
passes syntax checks and the failed-evaluator regression test; a clean end-to-end
runner invocation remains to be recorded before a tagged release.

## Interfaces and compatibility

`/car/lead_distance` has been replaced by `/car/lead_estimate` using
`auto_drive_sim_interfaces/msg/LeadEstimate`. Consumers must read `distance`,
`detected`, `valid`, and the source image timestamp. Rebuild both ROS packages.
The lead detector publishes clear observations as well as detected vehicles.
`lead_enabled=true` requires a valid observation within `lead_timeout_s`.
The demo enables this requirement when `lead_detector:=true`.

The evaluation bridge now consumes `/model/car/pose` and publishes
`/evaluation/ego_pose` as `geometry_msgs/msg/PoseStamped`. The evaluator scores
source timestamps and discards repeated or out-of-order poses. Perception and
control do not subscribe to this topic.

Each scenario-runner invocation creates a unique directory under
`artifacts/evaluations/` or the requested `RESULTS_DIR`. The printed path contains
its metadata and summary. Existing public result snapshots are not overwritten.
Cleanup targets the processes belonging to the run rather than every ROS or
Gazebo process on the machine.

## Remaining validation

Older dashed, shadow, fast, two-lane, and moving-lead result tables retain their
historical status. They have not been revalidated by the baseline runs above.
GitHub Actions is configured for the fast suite; its hosted run is pending a push.
