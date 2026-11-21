# macOS and Linux VM audit — 2026-11-22

The existing Lima VM boots and runs Gazebo, but its shadowed camera delivery
has gaps that exceed the controller's 0.3-second simulation-time freshness
limit. This is a measured runtime limitation. There is no evidence here of
an ARM emulation error, exhausted guest RAM, or a full guest disk.

## What I checked

I inspected the host, the saved Lima configuration, guest architecture and
resources, graphics driver, filesystem mounts, clock synchronization, source
revision, existing failed laps, and live camera timing. I temporarily started
the existing VM, then stopped it again. I did not change its settings, install
packages, stop unrelated Docker services, or change the controller's stale limit.

The host has 32 GiB RAM and eight CPU cores (six performance cores). Lima uses
Apple Virtualization (`vz`), ARM64, eight vCPUs, 12 GiB RAM, and a 40 GiB disk.
The guest reported about 11 GiB available at idle, no swap, no memory pressure,
and only 6.5 GiB used on its 38 GiB root filesystem. Its timezone is
Europe/Athens and NTP synchronization was enabled. Gazebo is version 8.15.0.

Docker's Linux VM exposes eight CPUs and about 7.65 GiB RAM. Twelve unrelated
Supabase containers remained running during the audit. The host had about
4.58 GiB of swap in use, but the live memory-pressure check reported 38% free;
swap occupancy alone does not establish active memory pressure or prove the
cause of a rendering stall. macOS reported no recorded thermal warning.

## Measured camera timing

| Runtime/profile | Wall Hz | Simulation Hz | Max simulation gap | Gaps > 0.3 s | p95 arrival age |
|---|---:|---:|---:|---:|---:|
| Lima, shadows | 8.53 | 8.90 | 0.396 s | 6 | 0.012 s |
| Lima, no shadows | 13.38 | 13.59 | 0.198 s | 0 | 0.007 s |
| Lima, shadows, half speed | 4.00 | 8.09 | 0.396 s | 2 | 0.007 s |
| Docker, shadows | 13.97 | 15.10 | 0.132 s | 0 | 0.015 s |
| Docker, no shadows | 14.23 | 15.10 | 0.132 s | 0 | 0.011 s |

Final samples had no duplicate image timestamps. Raw measurements are in
[`diagnostics/vm-audit.json`](diagnostics/vm-audit.json). Local scripts, worlds,
and logs are in `artifacts/vm-audit/`. These camera checks do not change the
historical VM lap failures into passing laps.

## Findings

### Software rendering is the main demonstrated bottleneck

The Lima guest has no `/dev/dri` device. Surfaceless EGL works with Mesa
llvmpipe, LLVM 20.1.2, OpenGL 4.5. X11/Wayland initialization fails because this
is a headless guest; that failure is not itself a broken camera driver.

The current Lima documentation lists GPU acceleration for its Krunkit backend,
not this VZ configuration: [Lima GPU support](https://lima-vm.io/docs/config/gpu/).
Docker's standard Linux-container GPU passthrough documentation applies to
Windows/WSL2, not this Mac setup: [Docker GPU support](https://docs.docker.com/desktop/features/gpu/).
The project Docker image explicitly sets `LIBGL_ALWAYS_SOFTWARE=1`.

Docker also runs through a Linux VM on macOS. Its historical passing laps do
not prove that virtualization is absent or that Gazebo uses the Apple GPU.
See [Docker's architecture explanation](https://docs.docker.com/desktop/setup/install/mac-permission-requirements/).

### Average FPS is not the acceptance criterion

A five-Hz stream averages a frame every 0.2 seconds, which can fit inside the
0.3-second limit. Uneven gaps, frame age, and the simulation clock determine
whether control stops. The older environment notes attributed failure too
strongly to average FPS alone.

I measured arrival wall time, source image timestamp, and current ROS simulation
time for each image. This distinguishes a slow wall-clock simulation from
missing updates in simulation time. The shadowed VM's frame-arrival age was
small, while its source-timestamp gaps crossed the limit. That points toward
render/update scheduling as the dominant issue in this static scene rather
than a large bridge queue delay. It does not rule out additional perception
processing or bridge delays in a complete driving run.

### The VM consumes the full host CPU allocation

Eight vCPUs are configured on an eight-core host, while Docker exposes another
eight virtual CPUs and macOS applications also run. This permits contention;
it is not proof that CPU allocation alone caused the failure. Earlier four-vCPU
results were slightly faster than eight-vCPU results, but those are not a
controlled scaling benchmark. More vCPUs should not be assumed to improve
llvmpipe performance. Compare four and six vCPUs under a consistent host load
before changing the supported configuration.

### The shared mounts overlap

Lima emitted an explicit warning that the home-directory mount overlaps the
writable project mount. The project mount currently resolves as writable
`virtiofs`; Lima says overlapping mounts are unsupported and can cause errors.
Use one unambiguous mount arrangement. Keep Linux build/install/log directories
on guest-local storage so Docker and Lima do not overwrite each other's build
outputs.

The original VM clone is on local ext4 storage, so the historical camera
failure cannot simply be attributed to shared-project I/O. The diagnostic
camera scenes were loaded through the shared project mount; steady-state
rendering does not require repeatedly reading the world file.

### The VM checkout is old

The guest-local checkout is at `002a1a1`, with modified historical result files.
The host base revision is `f8e50ce` plus current maintenance changes. The guest
has not been updated to the new stamped lead message or dedicated evaluator
pose topic. A VM result from that clone cannot validate today's host tree.
The camera benchmarks use identical host-supplied scenes and a common probe,
independent of either project's controller revision.

### The current camera feasibility gate is too weak for control readiness

`tools/camera_probe.py` checks image dimensions, contrast, and a minimum of ten
frames during ten seconds. It can pass a slow or irregular stream. It does not
measure maximum simulation-time gaps or enforce a freshness budget. Keep this
as an image-feasibility test, and add a separate sustained timing gate before
calling an environment ready for closed-loop driving.

## Benchmark method and limits

Each sample observes twenty wall-clock seconds after seven seconds of startup.
The scene is the committed static 640 × 360 camera check requesting 15 Hz.
Only shadow settings or the requested real-time factor change. ROS image and
clock topics are bridged in the same guest. Each final scene uses an isolated
Gazebo partition and process-group cleanup. Final samples check duplicate image
timestamps. The first no-shadow exploratory sample had duplicate bridge output
and is excluded.

The half-speed experiment kept shadows and the 0.3-second controller policy.
It still produced gaps over that policy's limit; lowering the real-time factor
alone is not demonstrated to solve the problem.

These are short static-scene measurements, not full laps or a guarantee of
long-run stability. Host application load was not frozen. The Mac has not
been compared against GPU-equipped native Linux. GUI usability, suspend/resume,
and cross-host ROS discovery were not tested. Simulation topics stayed inside
the guest; Foxglove's forwarded WebSocket remains the existing host-viewing path.

## Recommended next steps

1. Keep Docker as the supported baseline based on the recorded full laps.
2. Remove the overlapping Lima mount configuration in a deliberate VM setup change.
3. Sync the current source into a guest-local checkout and build it separately.
4. Test an explicit lightweight world with shadows off, preserving the 0.3-second
   freshness policy. Its camera result must be followed by all three full laps.
   Do not replace the shadowed release results with results from a different world.
5. Add a sustained timing gate recording simulation-time gap maxima, p95 arrival
   age, real-time factor, and stale events. Preserve the distinction between
   feasibility, timing readiness, and full driving acceptance.
6. If shadows and more realistic scenes are required, evaluate a GPU-capable
   backend or a Linux GPU host as a separate environment. Krunkit is a candidate
   to investigate, not a verified drop-in Gazebo solution here.
