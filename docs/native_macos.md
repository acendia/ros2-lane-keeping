# Native macOS / Metal experiment

This branch evaluates native ROS 2 Jazzy and Gazebo Harmonic on Apple Silicon.
Docker remains the supported Linux baseline. Native results must be measured
before this branch is considered for merging.

## Dependencies

The experiment uses community package `idesign0/ros2/ros2-jazzy` version
`26.11.20.9`. Its [formula](https://github.com/idesign0/homebrew-ros2/blob/main/Formula/ros2-jazzy.rb)
requires python.org framework Python 3.11. Homebrew Python is not interchangeable
with that binary bundle. The package ships Gazebo and supporting libraries;
do not mix another Gazebo installation into its environment.

Install the official Python 3.11 universal2 package, then the community ROS
bundle following its instructions. Run dependency installation explicitly with:

```bash
/Library/Frameworks/Python.framework/Versions/3.11/bin/python3 -m pip install --user \
  'numpy<2' 'empy==3.3.4' 'lark==1.1.1' catkin_pkg netifaces osrf-pycommon \
  PyYAML packaging psutil typeguard jinja2 lxml argcomplete typing_extensions \
  colcon-common-extensions cryptography pytest 'opencv-python==4.11.0.86'
```

`tools/native.sh` sources this bundle only for the command it runs. It does not
edit shell startup files or replace the machine's default Python. The wrapper
sets Gazebo to `127.0.0.1` and Cyclone DDS to `lo0` for a single-Mac run; it is
not the profile to use for networking with a physical robot. An alternate
bundle location can be supplied through `NATIVE_ROS_PREFIX`.

## Build and verify

Keep native build outputs separate from Linux outputs:

```bash
tools/native.sh colcon --log-base log-native build \
  --build-base build-native --install-base install-native --symlink-install
tools/native.sh python3 -m pytest -q tests
tools/native.sh python3 tools/native_check.py camera --duration 60
tools/native.sh python3 tools/native_check.py scenarios
```

The camera check retains shadows. The native launch omits Linux's EGL
`--headless-rendering` flag; Gazebo runs its server without a GUI client and
uses its macOS rendering path. Check the Ogre log for Metal: a successful
window or camera topic alone does not establish GPU use.

Each validation writes fresh logs, source provenance, and measurements beneath
`artifacts/native-validation/`. Scenario checks retain the 0.3-second freshness
limit and existing footprint and p95 targets. Results are not merged into the
historical Docker directories.

## Acceptance

Require actual Metal renderer evidence, nonblank camera frames, at least 14 Hz
in simulation time with no gaps or arrival ages over 0.3 s in the camera check,
and successful full laps from all three starting offsets. Compare repeated
runs and renderer-dependent perception behavior before making a performance
or reliability claim. The Linux shell-runner regression test is skipped on
macOS; Docker CI continues to run it.

## Installation observations

The bundle occupies approximately 10.3 GiB. First-use loading can take longer
while macOS validates its libraries. The camera runner waits for a real image
with a bounded timeout rather than assuming that eight seconds is enough.
Do not start validation until Homebrew installation completes.

This bundle emits missing-setup warnings for `kuka_external_control_sdk_iiqka`,
`off_highway_premium_radar`, and `ros2_medkit_gateway`. Those packages are not
used by this project. The project build and native test suite must still pass.
The framework Python dependencies are installed in the user account, without
unlinking other Python versions or disabling macOS security protections.

## First measured result — 2026-11-22

On an Apple M1 Pro, macOS 26.6.2, Python 3.11.9, and community ROS bundle
26.11.20.9 (Gazebo 8.15.0), Ogre selected the Apple M1 Pro Metal device.
The shadowed 640 × 360 camera check produced 900 frames in 60 seconds:
14.991 Hz wall, 15.000 Hz simulation, real-time factor 0.9994, maximum
simulation-time gap 0.067 s, maximum arrival age 0.007 s, and no stale gaps.

| Scenario | Lap time | p95 lateral error | Footprint crossing | Stops |
|---|---:|---:|---|---:|
| Center | 88.40 s | 0.2306 m | No | 0 |
| Left +0.3 m | 88.40 s | 0.2525 m | No | 0 |
| Right −0.3 m | 88.40 s | 0.2533 m | No | 0 |

The complete native runner exited successfully. See
[`results-native/`](../results-native/) for JSON, CSV, camera evidence, and
run-start provenance. These were working-tree measurements based on 59329d8
with this branch's native changes. The source fingerprint records the files
at run start; later documentation and wrapper sourcing cleanup are not part
of that snapshot.

The previous Docker static-camera audit measured 13.97 Hz wall, a 0.925
real-time factor, and a 0.132 s maximum simulation-time gap over 20 seconds.
Its baseline laps passed too. The native result demonstrates steadier camera
timing in these samples and similar lane accuracy, not a general speedup
claim. Camera update rate is capped at 15 Hz in both profiles. CPU load, power,
and sustained complex-world performance were not compared.

Native tests: 89 passed, one Linux-only runner test skipped. Linux Docker
regression: both packages rebuilt and all 90 tests passed. Earlier attempts
failed during incomplete installation/cold startup and with default network
discovery. The successful run used the wrapper's loopback settings. The
community bundle still emits unrelated setup warnings. Forced teardown can
print an rclpy context-shutdown traceback after evaluation has finished; it
did not interrupt any of the scored laps.

This remains experimental. Repeat cold starts, longer runs, the dashed and
shadow worlds, lead following, and the GUI before considering a merge to main.

## Clean-revision validation — 2026-11-22

The supervised checks below were collected from committed revision
`148c58f0fed84a9655f4cad89262210f9fe2f51d`, with a clean working tree at the
start of each run. They supersede the historical snapshot above as the
reproducible baseline for this branch.

Ten independent cold starts passed their readiness, image, Metal-device, and
process-cleanup checks. The 60-second camera run collected 900 frames at
14.996 Hz wall rate and 15.000 Hz simulation rate, with a 0.99975 observed
real-time factor, a 0.067 s maximum simulation-time gap, and a 0.003 s p95
arrival age.

| Scenario | Lap time | p95 lateral error | Footprint crossing | Stops |
|---|---:|---:|---|---:|
| Center | 88.40 s | 0.2306 m | No | 0 |
| Left +0.3 m | 88.40 s | 0.2311 m | No | 0 |
| Right −0.3 m | 88.40 s | 0.2314 m | No | 0 |

The published records include source hashes, effective settings, environment
versions, input hashes, renderer selection, readiness acknowledgements, and
per-step cleanup status:
[`startup`](../results-native-hardening/startup-2026-11-22/),
[`camera`](../results-native-hardening/camera-2026-11-22/), and
[`scenarios`](../results-native-hardening/scenarios-2026-11-22/).

## Supervised validation (milestones 1–2)

The runner checks the exact qualified bundle, native Python imports, and the
installed scenario assets before launching a simulator. Environment details,
package versions, named source hashes, effective settings, and input hashes are
recorded alongside each result. A changed installed asset requires a rebuild.
An intentional bundle upgrade can be qualified using `NATIVE_ROS_EXPECTED_VERSION`;
changing that value alone does not establish compatibility.

```bash
tools/native.sh python3 tools/native_check.py preflight
tools/native.sh python3 tools/native_check.py startup --repetitions 10 --require-clean
tools/native.sh python3 tools/native_check.py camera --duration 60 --require-clean
tools/native.sh python3 tools/native_check.py scenarios --require-clean
```

`--domain` (or `NATIVE_ROS_DOMAIN_ID`) selects a local ROS domain in 0–100.
Checks reserve their domain and serialize access to Ogre's shared renderer log.
Do not run an unrelated native Gazebo instance during validation. A Metal pass
requires fresh log contents held open by an owned process and an actual device
selection; loading the Metal plugin is insufficient. An explicit software-rendering
request is rejected rather than silently removed.

Readiness requires an advancing simulation clock and current image data; driving
also requires a valid lane estimate and fresh odometry. The evaluator writes an
acknowledgement only after it processes its first scored pose. These bounded
checks replace fixed sleeps. Defaults allow 45 wall seconds for startup and
420 wall seconds for a scenario. `--startup-timeout` and `--scenario-timeout`
control those deadlines, independently of the unchanged 0.3 s input freshness
policy and 130 simulated-second scoring timeout.

Each run produces `validation.json` and a generated `summary.md`, including on
failure or interruption. Logs stay with that invocation. Ctrl-C returns 130;
failed or timed-out checks return nonzero. Cleanup signals ROS launcher leaders
first, then escalates to owned groups and known descendants. Expected context
shutdown is handled by the nodes; runtime exceptions with a live context remain
errors. A successful check also requires completed process cleanup.

To export a successful clean-revision run into a **new** public snapshot:

```bash
python3 tools/native_report.py artifacts/native-validation/<run> \
  --output results-native-hardening/<snapshot>
```

Export rejects failed, interrupted, dirty, incomplete, or mismatched records and
never overwrites an existing snapshot. It copies JSON/CSV evidence and camera
images, extracts renderer selection, and omits raw process logs and personal
home paths. Historical `results-native/` records retain their original provenance.
