# Track

The first-release track is an original oval generated from
[`src/auto_drive_sim/config/track.yaml`](../src/auto_drive_sim/config/track.yaml).
It has two 20 m straights and two semicircular ends of 15 m radius. The clear
lane is 3.5 m wide between the inner edges of two continuous white boundary
strips.

Track version 2 keeps the same oval and start pose but uses dashed boundaries
with worn segments. It is generated from
[`src/auto_drive_sim/config/track_markings.yaml`](../src/auto_drive_sim/config/track_markings.yaml).

Both tracks are deliberately simple. They have no other lanes, no
intersections, no traffic, and no obstacles.

## Dimensions

| Item | Value |
|---|---|
| Straight length | 20.0 m each |
| Curve radius | 15.0 m |
| Centerline length | 134.24778 m |
| Lane width | 3.5 m between marking inner edges |
| Boundary marking width | 0.12 m |
| Shoulder each side | 0.75 m |
| Asphalt width | 5.0 m |
| Centerline sample spacing | 0.1 m |
| Centerline samples | 1342 |

Track version 2 changes only the markings:

| Item | Value |
|---|---|
| Dash length | 3.0 m |
| Gap length | 0.75 m |
| Boundary marking width | 0.16 m |
| Worn pattern | every third dash |
| Worn gap | 0.18 m split in the middle of the dash |
| Worn width | 75% of the normal marking width |
| Worn colour | 0.78, 0.78, 0.74 |
| Marking pieces per side | 48 |

## Coordinate frame

- The world origin is at the center of the oval at ground level.
- `+X` points east, `+Y` points north, and `+Z` points up.
- The centerline starts at `(-10.0, 15.0)` with yaw `0.0` and runs clockwise:
  - top straight from `x = -10` to `x = +10` at `y = +15`
  - east semicircle centered at `(+10, 0)` with radius 15 m
  - bottom straight from `x = +10` to `x = -10` at `y = -15`
  - west semicircle centered at `(-10, 0)` with radius 15 m
- The start pose from `track_meta.json` is `(-10.0, 15.0, 0.05)` with yaw
  `0.0`. The vehicle will face east along the top straight.
- The ground plane is at `z = -0.02`, the asphalt visual at `z = 0.005`, and
  the white marking visual at `z = 0.012`.

The road meshes are visual only. The physics ground is the flat plane, so a
future vehicle drives on the ground while the painted surface is slightly above
it.

`centerline.csv` stores one row per sample:

```text
s_m,x_m,y_m,yaw_rad,curvature_1_per_m
```

`yaw_rad` is counter-clockwise from `+X`. The clockwise oval therefore has
negative curvature on both curves and zero curvature on the straights.

## Generated assets

```text
src/auto_drive_sim/models/road/
├── model.config
├── model.sdf
├── track_meta.json
├── centerline.csv
└── meshes/
    ├── asphalt.obj
    ├── left_boundary.obj
    ├── right_boundary.obj
    └── road.mtl

src/auto_drive_sim/models/road_markings/
├── model.config
├── model.sdf
├── track_meta.json
├── centerline.csv
└── meshes/
    ├── asphalt.obj
    ├── left_boundary.obj
    ├── right_boundary.obj
    └── road.mtl
```

For track version 1, each boundary is one continuous closed triangle strip, not
one mesh per straight or curve, so there are no mesh joints at the
straight-to-curve transitions. For track version 2, each boundary object holds
one group per dash or worn piece with its own material.

## Regenerating the assets

Run the generator from the repository root:

```bash
python3 tools/generate_track.py
python3 tools/generate_track.py \
  --config src/auto_drive_sim/config/track_markings.yaml \
  --output src/auto_drive_sim/models/road_markings
```

Inside the Docker environment:

```bash
./docker/run.sh bash -lc 'python3 tools/generate_track.py'
```

Then run the geometry tests:

```bash
python3 -m pytest -q tests/test_track_geometry.py
```

The tests check the centerline length and closure, the start pose, the 3.5 m
lane width, the asphalt bounds and height, the curvature values, the dashed and
worn pattern, and that two independent generations are byte-identical. They
also compare both committed asset sets with fresh generations and check that
every mesh face index exists, so stale or broken assets fail the suite.

## Viewing the world

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch auto_drive_sim track_preview.launch.py gui:=false
```

The launch sets `GZ_SIM_RESOURCE_PATH` to the installed model directory and
bridges `/clock`. To load the dashed and worn road, launch the demo with
`world:=car_world_markings.sdf`. `gui:=true` starts the Gazebo client on a
machine with a display; the Docker environment on this Mac has no display.

## Two-lane variant

Track version 3 in
[`src/auto_drive_sim/config/track_two_lane.yaml`](../src/auto_drive_sim/config/track_two_lane.yaml)
keeps the same oval but adds a second 3.5 m lane. The three marking centres are
at road-centre offsets −3.62 m, 0 m, and +3.62 m. The car drives in the right
lane, whose centreline is 1.81 m right of the road centre. Its lane length is
122.887 m, not the road-centre length of 134.248 m.

The generated assets are in
[`src/auto_drive_sim/models/road_two_lane/`](../src/auto_drive_sim/models/road_two_lane/).
The world is `car_world_two_lane.sdf`, with the car starting at
`(-10.0, 13.19)`.

Regenerate it with:

```bash
python3 tools/generate_track.py \
  --config src/auto_drive_sim/config/track_two_lane.yaml \
  --output src/auto_drive_sim/models/road_two_lane
```

Run the right-lane check:

```bash
./docker/run.sh bash -lc 'WORLD=car_world_two_lane.sdf \
  START_Y=13.19 LANE_CENTER_Y=13.19 LANE_STRATEGY=rightmost \
  CENTERLINE_PATH=$PWD/install/auto_drive_sim/share/auto_drive_sim/models/road_two_lane/centerline.csv \
  RESULTS_DIR=results-two-lane RUN_DIR=artifacts/scenarios-two-lane \
  tools/run_scenarios.sh'
```

All three scenarios pass: 83.62 s, 83.64 s, and 83.62 s, with p95 errors of
0.269 m, 0.270 m, and 0.270 m. There are no crossings and no stops. Results are
in [`results-two-lane/`](../results-two-lane/).

## Shadow test world

`car_world_lighting.sdf` keeps the dashed and worn road but lowers the ambient
light, moves the sun low in the south, and adds four shadow-casting buildings
beside the straight and the east curve. The static camera set is
[`tests/gazebo/lighting_frames.sdf`](../tests/gazebo/lighting_frames.sdf).

The six labeled frames in [`tests/frames_lighting/`](../tests/frames_lighting/)
pass. All three driving scenarios passed in the recorded run, with p95 errors of
0.230 m (center), 0.249 m (left), and 0.300 m (right), and 8–10 stop events.
Repeat center runs varied, so this is not a repeatable check yet. Results are
in [`results-lighting/`](../results-lighting/).

## Camera verification

Run:

```bash
tools/run_road_camera_check.sh
```

The test world places four static cameras: the start straight, the east curve
entry, and the east and west curve apexes. Each camera is 640 × 360, 15 Hz,
with a 60° horizontal field of view.

![Four camera views of the oval](images/track_camera_views.png)

Results from the 2026-11-13 Docker run:

| View | Frames in 6 s | Wall rate | Timestamp rate | Probe |
|---|---:|---:|---:|---|
| Start straight | 40 | 6.76 Hz | 9.85 Hz | valid |
| East curve entry | 35 | 6.28 Hz | 9.20 Hz | valid |
| East curve apex | 42 | 7.03 Hz | 10.35 Hz | valid |
| West curve apex | 57 | 10.12 Hz | 14.89 Hz | valid |

All four images show both boundary lines. The lines stay continuous through the
straight-to-curve transitions, which supports the no-seam requirement. Gazebo
reported no missing or failed assets.

The lower rates come from rendering four 640 × 360 cameras at once with the
CPU-only Mesa llvmpipe backend. The single-camera Milestone 0 check produced
15.15 Hz in the same container.

The dashed and worn variant has its own static frame set in
[`tests/frames_markings/`](../tests/frames_markings/), captured from six camera
poses with the same intrinsics. Those frames are used by the perception tests.

At the distant outer asphalt edge, the image shows a faint dashed fringe. It
appears to be subpixel aliasing of the road edge under software rendering, not
a gap in a lane marking; the white marking itself is continuous and inset from
that edge. Later perception work should not treat it as a lane feature.

## Limitations

- One lane only, with no center line.
- The dashed variant has 3 m dashes, 0.75 m gaps, and a fixed light wear
  pattern. Long gaps, heavy wear, and wet markings are not tested.
- The shadow world passes with 8–10 stop events. A longer perception loss still
  stops the car.
- The two-lane variant uses solid boundaries and the right lane only. There is
  no lane change, intersections, or traffic.
- Flat road with uniform materials and no texture.
- The road is visual geometry without collision; the car uses the ground plane
  for physics.
- The Docker GUI path is untested.
