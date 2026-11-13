# Lane perception

The perception node uses only the forward camera image. It does not read
odometry, the simulator pose, or the track centerline.

The live node subscribes to `/car/camera/image_raw`, publishes
`auto_drive_sim_interfaces/msg/LaneEstimate` on `/car/lane_estimate`, and
publishes an overlay on `/car/debug_image`.

## Camera and ground plane

The camera is fixed at 1.3 m above the road, pitched 0.05 rad down, 1.4 m
ahead of the model origin. It produces 640 × 360 images with a 90° horizontal
field of view.

| Intrinsic | Value |
|---|---|
| `fx` | 320.00 px |
| `fy` | 320.00 px |
| `cx` | 320 px |
| `cy` | 180 px |

For a pixel `(u, v)`, the normalized camera coordinates are:

```text
a = (u - cx) / fx
b = (v - cy) / fy
```

In the camera frame with X forward, Y left, and Z up, the ray is
`(1, -a, -b)`. Rotating it by the pitch `p` gives a world-aligned direction.
The ground intersection at camera height `h = 1.3 m` is:

```text
t   = -h / ray_z
X   = t * ray_x
Y   = t * ray_y
```

Only rays with `ray_z < 0` reach the ground. The ROI is 3.5–12 m forward and
±5 m lateral.

The lookahead target is **4 m ahead of the camera**, which is 5.4 m ahead of
the model origin. The published target uses vehicle coordinates, so
`lookahead_target.x = 5.4`.

## Pipeline

![Perception stages](images/perception_pipeline.png)

1. **Mask the markings.** Convert to HSV and keep bright, low-saturation
   pixels: `V ≥ 180`, `S ≤ 60`. The road is grey and the markings are bright.
2. **Project to the ground.** Use the fixed camera model to turn each mask
   pixel into a ground point. The sky produces no ground points, so it is
   ignored naturally.
3. **Cut the road ROI.** Keep points from 3.5 to 12 m forward and within ±5 m
   lateral.
4. **Find the two markings.** Take connected white components with enough
   pixels and ground points. Keep the two largest components. Assign the
   higher-lateral component to the left boundary and the other to the right.
5. **Fit each boundary.** Fit a quadratic `y(x)` with three residual-rejection
   passes. Require at least 50 inliers, a 3 m forward span, and RMS error below
   0.15 m.
6. **Validate and estimate.** Compute the lane width at the 4 m lookahead.
   Reject a pair that is swapped, narrower than 2.8 m, wider than 4.5 m, or
   has a center more than 2.5 m from the camera axis. The lateral error is the
   lane center at the 4 m camera lookahead.

The confidence combines boundary point count, fit RMS, and width error. It is
a relative quality score, not a probability.

The node rejects `valid=false` for these reasons:

| Reason | Cause |
|---|---|
| `no_lane_markings` | No marking points reach the ground ROI |
| `one_boundary` | Only one boundary was found |
| `sparse_boundaries` | Too few inliers, short span, or poor fit |
| `swapped_boundaries` | Left fit is not left of the right fit |
| `lane_too_narrow` / `lane_too_wide` | Width outside 2.8–4.5 m |
| `lane_offset_too_large` | Lane center more than 2.5 m from camera axis |
| `stale_image` | Image timestamp older than 0.3 s |

## Saved frame tests

The test frames are in [`tests/frames/`](../tests/frames/). They were captured
from four static cameras using the same camera model as the car.

![Saved perception frames](images/perception_test_frames.png)

| Frame | Valid | Confidence | Lateral error | Line-center width |
|---|---:|---:|---:|---:|
| Straight | yes | 0.895 | 0.006 m | 3.672 m |
| Right curve | yes | 0.881 | +1.083 m | 3.702 m |
| Left curve | yes | 0.880 | −1.071 m | 3.701 m |
| No line | no | 0.000 | — | — |

The right-curve error is positive because the lane is to the left of the
camera aim after the camera is pointed into the right bend. The left curve is
the opposite.

Additional rejection tests cover a blank frame, a frame with one boundary
removed, narrow and wide lane fits, swapped boundaries, sparse points, and
stale timestamps. The test suite has 42 tests.

Run the perception tests:

```bash
./docker/run.sh bash -lc 'python3 -m pytest -q tests/test_lane_perception.py'
```

Recapture the saved frames:

```bash
./docker/run.sh bash -lc 'tools/capture_perception_frames.sh'
```

## Live check

Start the world, bridge, and perception node:

```bash
./docker/run.sh bash -lc \
  'source install/setup.bash && ros2 launch auto_drive_sim perception.launch.py gui:=false'
```

At the start pose the live estimate was:

```text
valid: true
confidence: 0.868
reason: ""
lateral_error: 0.0062
lookahead_target:
  x: 5.4
  y: 0.0062
```

The debug image carried 38 frames in 4 s at 640 × 360, 12.17 Hz wall clock,
and 14.75 Hz message timestamps.

![Live debug overlay](images/perception_live_debug.png)

The green line is the fitted left boundary, the red line is the fitted right
boundary, and the yellow dot is the lookahead target.

## Revisions

The first live autonomy run stopped at the east curve because the 60° camera
saw only one boundary. I widened the camera to 90° and changed the boundary
extraction from per-distance pairing to two connected marking components. The
same run also showed that width measured at 4, 6, and 8 m grows on a curve, so
the width check now uses only the 4 m target.

## Limitations

- The method assumes a flat road and a fixed camera pitch. Body pitch from
  braking or acceleration will move the ground projection.
- Only continuous white markings are handled. Dashed, worn, shadowed, or wet
  markings are later work.
- The mark threshold is tuned for this scene. A different road material or
  lighting needs a new threshold.
- The camera cannot see the ground closer than about 3.4 m, so the lookahead is
  defined from the camera rather than the vehicle origin.
- `ros_gz_bridge` sometimes exits with a signal when the launch is interrupted
  in Docker. It does not affect the running estimate or the saved results.
