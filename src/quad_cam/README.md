# quad_cam

Real Raspberry Pi **Camera Module 3 Wide** (`imx708_wide`) publisher for the IBVS stack.

Publishes `sensor_msgs/Image` (`bgr8`) on `/quad/camera/image_raw`, ISP-downscaled to
**1152x648** — the `module3wide_2304` preset's render size in
`quad_gz_sim/config/cameras.yaml` — so `image_features` runs unchanged from simulation
(`fx`, `aD` and the distortion model all follow the declared size).

## Dependency: picamera2 (out-of-band)

The node uses **picamera2** + the RaspberryPi `libcamera` fork + the PiSP IPA. On Ubuntu
24.04 these are not in apt; they are built from source to `/usr/local` by
`tools/setup_pi5_ubuntu_camera.sh` and bridged onto the system `python3` that ROS Jazzy
runs on. There is no rosdep key, so picamera2 is **not** in `package.xml` — `rosdep
install` will not pull it and `colcon build` does not need it. Verify it resolves:

```
source /opt/ros/jazzy/setup.bash
python3 -c "import picamera2, libcamera; print('ok')"
python3 /home/quad/EI-DSoni/camera_module3_wide_test.py --doctor
```

## Run

```
colcon build --packages-select quad_cam
source install/setup.bash
ros2 launch quad_cam camera.launch.py
```

Then pair with the estimation chain (see the header of `launch/camera.launch.py` for the
exact `estimation.launch.py` arguments).

## Key parameters

| param | default | note |
| --- | --- | --- |
| `width` / `height` | `1152` / `648` | published size (ISP downscale) |
| `sensor_width` / `sensor_height` | `2304` / `1296` | pinned sensor mode (full 102 deg FOV, ~56 fps) |
| `framerate` | `0.0` | `0` = sensor mode max; kept above the 50 Hz loop deliberately |
| `image_qos` | `reliable` | `reliable` matches `image_features` today; `sensor_data` = BEST_EFFORT |
| `hflip` / `vflip` | `true` / `true` | sensor reports `Rotation: 180`; **verify on the bench** |
| `af_mode` / `lens_position` | `continuous` / `0.83` | use `manual` for flight (0.83 dioptres ~ 1.2 m) |
| `publish_camera_info` | `false` | off — `image_features` uses its own intrinsics |

## Testing

```
ros2 run quad_cam probe_latency   # rate, frame age (stamp vs receive), inter-frame jitter
ros2 run quad_cam probe_detect    # detectMarkers p50/p95/p99 + which IDs / lock rate
```
`probe_detect` runs exactly the ArUco call `image_features` does; its p99 is the
`first-flight-checklist.md` go/no-go number and must sit under the 20 ms loop budget at
the resolution actually published. Both take `-p image_qos:=` and `-p image_topic:=`.

## Reliability

Capture and publish run on separate threads with a single newest-frame slot between them
(pattern from `TE3002B_mcrChallenge/pzb_camera/camera_raw_publisher.py`): a stalled
subscriber drops frames, it never back-pressures capture. `capture_request(wait=...)`
timeouts trip a stop/start recovery, then a FATAL shutdown after `max_failures`. A 5 s
log line reports delivered fps, p95 inter-frame gap, and drop count.
