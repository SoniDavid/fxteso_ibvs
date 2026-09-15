# Running on hardware

The Raspberry Pi / real UAV side of [RUNNING.md](RUNNING.md); shared setup, the launch argument
reference, and camera presets live there. For SITL and the dev desktop, see
[RUNNING_SIM.md](RUNNING_SIM.md).

## Companion computer (Raspberry Pi 5, Ubuntu 24.04)

The Pi builds every package except `quad_gz_sim`, which needs the Gazebo toolchain. The camera preset table lives in `quad_description` so the Pi reads the same one the simulator renders from.

```sh
# 1. Camera stack. Ubuntu's libcamera has no PiSP support; this builds Raspberry Pi's fork,
#    libpisp and picamera2 (20-45 min). picamera2 has no rosdep key, so it is not in package.xml.
bash tools/setup_pi5_ubuntu_camera.sh
python3 tools/camera_module3_wide_test.py --doctor

# 2. OpenCV + cv_bridge, as in RUNNING.md, with the Pi's CPU_FLAGS

# 3. MicroXRCEAgent on PATH - hardware.launch.py runs `MicroXRCEAgent serial`
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build
cmake --build Micro-XRCE-DDS-Agent/build -j$(nproc)
sudo cmake --install Micro-XRCE-DDS-Agent/build && sudo ldconfig

# 4. The workspace, without the simulator and without sim_pilot (it arms the aircraft)
git submodule update --init src/px4_msgs
colcon build --symlink-install --packages-skip quad_gz_sim \
    --allow-overriding cv_bridge image_geometry --cmake-args -DBUILD_SIM_PILOT=OFF

# Sourcing - two lines on the Pi
source /opt/ros/jazzy/setup.bash
source ~/[path_to_project]/fxteso_ibvs/install/setup.bash
```

## Launch Arguments Reference

These add to the shared reference in [RUNNING.md](RUNNING.md#launch-arguments-reference), which
`hardware.launch.py` and `bench.launch.py` both build on.

### Hardware (`hardware.launch.py`)
- `quad_mass:=<float>` — Weighed as flown, battery in (default: `2.0`, the simulated F450's).
- `hover_thrust:=<float>` — `MPC_THR_HOVER` read off a real hover (default: `0.60`).
- `camera_driver:=<bool>` — Start `quad_cam`'s picamera2 driver; `false` expects an external one on `camera_topic` (default: `true`).
- `camera_width:=<int>`, `camera_height:=<int>` — What the driver publishes; empty is the preset's render size, 1152 × 648 for `module3wide_2304` (default: empty).
- `camera_topic:=<str>`, `camera_info_topic:=<str>` — (defaults: `/quad/camera/image_raw`, `/quad/camera/camera_info`).
- `sensor_qos:=<bool>` — BEST_EFFORT on the driver and on `image_features` together (default: `true`).
- `camera_distortion:="[k1, k2, p1, p2, k3]"` — From a real calibration; the preset's are an assumption (default: zeros).
- `af_mode:=<manual|continuous>`, `lens_position:=<float>` — Focus; 0.83 dioptres is ~1.2 m (defaults: `manual`, `0.83`).
- `camera_cpu:=<core>` — Pin `camera_node` (default: `2`).
- `agent:=<bool>`, `agent_device:=<path>`, `agent_baud:=<int>` — MicroXRCEAgent on the flight controller's UART (defaults: `true`, `/dev/ttyAMA0`, `921600`).
- `consent_rc_aux:=<0..6>` — RC aux channel that grants the handover; needs `RC_MAP_AUXn`, `0` = `~/handover` service only (default: `0`).
- `handover_tolerance:=<float>` — Altitude band the bridge waits for (default: `0.15`).
- `frame_yaw_offset:=<float>` — Workspace datum; zero, unlike SITL's −π/2 (default: `0.0`).
- `bag_prefix:=<str>` — Bag name prefix under `bags/` (default: `hw`).

### Bench (`bench.launch.py`)
- `load:=<synthetic|plate|none>` — What drives the chain without PX4 (default: `synthetic`).
- `attitude_controller:=<bool>` — Add `att_ctrl` to the load (default: `false`).
- `image_qos:=<sensor_data|reliable>` — Camera and `image_features` QoS together (default: `sensor_data`).
- `camera`, `zD`, `target_scale`, `marker_dict`, `quad_mass`, `af_mode`, `lens_position`, `camera_cpu` and the vision arguments — as in RUNNING.md.

## Running on the UAV

On the RPi5, `hardware.launch.py` starts the camera driver, the
MicroXRCEAgent link to the flight controller, the PX4 adapters and gates, and the estimation and control chain; nothing simulated:

```sh
ros2 launch quad_px4 hardware.launch.py \
    quad_mass:=<weighed, battery in> hover_thrust:=<measured MPC_THR_HOVER \ consent_rc_aux:=1
```

The sequence is the SITL one with a person in it: the pilot arms and flies to `takeoff_alt` → `px4_takeoff_gate` starts the estimators → `ibvs_gate` holds for a marker lock → the pilot grants consent → `pos_ctrl` starts and the bridge switches to OFFBOARD. A bag is recorded by default.

Four things differ from SITL, all silent if wrong. `use_sim_time` is false, so with no `/clock` the stack hangs rather than slows if it's left on. `frame_yaw_offset` is `0`, not the sim's `-pi/2`. The camera is subscribed BEST_EFFORT, at 1152 × 648 by default — the driver and `image_features` read their size from one place, and `image_features` refuses a frame of any other size. And the handover needs a person: `require_consent` is fixed true, granted by the RC aux switch or `~/handover`, and withdrawable at any time.

Every process also runs with `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`. A 2.24 MB frame does not fit Fast DDS's default 512 KiB shared-memory segment, and under load it is dropped with no error.

Before flying:

- Load `params/hardware.params` onto the flight controller and resolve its TODOs (declination,
  `MPC_THR_HOVER`, `RC_MAP_AUX1`). It is **not** derived from the SITL airframe, which a real flight
  controller cannot load and which disables failsafes.
- Calibrate the lens (`python3 tools/camera_module3_wide_test.py --calib 20`) and pass the result
  as `camera_distortion`. Check the image is upright; the driver flips it 180° by default.
- Build without the simulated pilot — it arms the aircraft and flies on Gazebo ground truth
  (`-DBUILD_SIM_PILOT=OFF`, already in the Pi build above).

Another camera driver can stand in for `quad_cam`. Start it with the same transport profile, or `image_features` receives nothing: a `LARGE_DATA` subscriber did not receive 2.24 MB frames from a
default-transport publisher in SITL.

```sh
FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA ros2 run <driver package> <driver>
ros2 launch quad_px4 hardware.launch.py camera_driver:=false \
    camera_topic:=/camera/image_raw camera_width:=<published> camera_height:=<published>
```

## Bench testing on the Pi

Without a flight controller, `bench.launch.py` runs the real camera and the estimation and control
chain, fed by `bench_feeder` in place of PX4 and the plant. This is where the 50 Hz budget is
measured.

The camera alone first:

```sh
ros2 launch quad_cam camera.launch.py image_qos:=sensor_data
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA   # the probes must share camera_node's transport profile
ros2 run quad_cam probe_latency --ros-args -p image_qos:=sensor_data   # rate, frame age, jitter
ros2 run quad_cam probe_detect --ros-args -p image_qos:=sensor_data    # OpenCV detect timing and lock rate
```

Against SITL's camera (`/quad/camera/image_distorted`) leave the variable unset — the simulator publishes with the default profile.

`probe_detect` times Python's OpenCV detector, so treat it as the reference arm. The detector actually flown is measured by `image_features` itself, in the bench below.

Then the stack, one-shot — launch, `probe_bench`, and a `bench_<timestamp>.log` report:

```sh
./src/quad_cam/scripts/bench.sh --load synthetic --duration 120
./src/quad_cam/scripts/bench.sh --load synthetic --att true --duration 120   # + att_ctrl
./src/quad_cam/scripts/bench.sh --load plate --duration 120                   # real ArUco plate
./src/quad_cam/scripts/bench.sh --load synthetic --backend opencv             # detector A/B
./src/quad_cam/scripts/bench.sh --load synthetic --backend hybrid --nano-ec 0.3 --nano-border 0.35 --cv-threads 1
./src/quad_cam/scripts/bench.sh --load synthetic --backend nano --nano-ec 0.3 --nano-border 0.35       # tuned nano alone

# or by hand
ros2 launch quad_cam bench.launch.py load:=synthetic
ros2 run quad_cam probe_bench
```

- `load:=synthetic` needs no printed target: `bench_feeder` fakes the plant, the target and a marker
  lock; `image_features` still runs for its real cost, its outputs moved to `*_probe`.
- `load:=plate` drives the real vision chain from a printed `{4,6,8,10}` 7×7 plate ~1.2 m from the lens.
- `load:=none` leaves `fixed_eso` and `pos_ctrl` idle.

A loop passes when `achieved >= 0.98 * target` with few overruns. The report collects each node's
`loop:` line, `image_features`' `detectMarkers (nano)` timing, and `probe_bench`'s per-process CPU,
SoC temperature and throttle flags.

`camera_node` captures and publishes on separate threads with a single newest-frame slot between
them, so a slow subscriber drops frames rather than stalling capture. Capture timeouts trigger a
stop/start recovery (at most one per `restart_cooldown_s`), and a FATAL shutdown after
`max_failures`. Every 5 s it logs delivered fps, the p95 inter-frame gap and the drop count.
