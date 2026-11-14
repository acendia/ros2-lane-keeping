# Live Foxglove interface

The Docker image includes `foxglove_bridge` 3.6.0. The demo can start it with
`foxglove:=true`, and `docker/run.sh` publishes port 8765 on the host.

## Start the demo

```bash
./docker/run.sh bash -lc \
  'source install/setup.bash && ros2 launch auto_drive_sim demo.launch.py gui:=false foxglove:=true'
```

The car starts driving immediately. The bridge listens on
`0.0.0.0:8765` inside the container and is available at
`ws://localhost:8765` on the Mac.

## Connect Foxglove

1. Open `Foxglove.app`.
2. Choose **Open connection**.
3. Enter `ws://localhost:8765`.
4. Connect.

The bridge advertises the live topics automatically.

## Suggested panels

| Panel | Topic | Field |
|---|---|---|
| Image | `/car/camera/image_raw` | raw forward camera |
| Image | `/car/debug_image` | mask tint, boundary fits, target |
| Plot | `/car/lane_estimate` | `lateral_error`, `confidence` |
| Plot | `/car/cmd_vel` | `linear.x`, `angular.z` |
| Raw Messages | `/car/lane_estimate` | full estimate |

A useful layout is the raw camera and debug overlay side by side, with the two
plots underneath. The plot x-axis follows simulated time.

The `/car/debug_image` panel shows the thresholded mask tinted green, the left
fit in green, the right fit in red, and the lookahead target as a yellow dot.

## Notes

- The bridge is optional. `demo.launch.py` defaults to `foxglove:=false`.
- The interface is read-only. Perception and control do not use Foxglove.
- If port 8765 is already in use, set `FOXGLOVE_PORT` for the Docker host port.
  The container-side port stays 8765.
- On a native Ubuntu install, add `ros-jazzy-foxglove-bridge` and run the same
  launch command.
- The Gazebo GUI is separate and still untested in Docker. Foxglove shows the
  camera and telemetry, not the Gazebo 3D client.
