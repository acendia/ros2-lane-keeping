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
