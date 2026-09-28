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
- `quad_mass:=<float>` — Weighed as flown, battery in (default: `2.0`, the thesis vehicle's; the 3S build is ~1.3 kg by parts list).
- `hover_thrust:=<float>` — `MPC_THR_HOVER` read off a real hover (default: `0.60`; SITL predicts ~0.69 for 3S at 1.30 kg).
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

### Vicon (`hardware.launch.py`)
- `vicon:=<bool>` — Start `vicon_px4_bridge` and aid EKF2 with mocap (default: `false`).
- `vicon_topic:=<str>` — The receiver's `PoseStamped` (default: `/vicon/quad/quad`).
- `aiding_policy:=<gated|always|oneshot|manual>` — `gated` aids EKF2 only while the loop does
  **not** have the aircraft, so the scored OFFBOARD window is unaided. `always` is for bring-up
  and shadow flights. `manual` is the `~/vicon_aiding` service (default: `gated`).
- `vicon_frame:=<enu|ned>`, `body_frame:=<flu|frd>` — The wand calibration and the object
  template's axes. Both are assumptions until confirmed by hand; see the ladder below
  (defaults: `enu`, `flu`).
- `vicon_yaw_offset:=<float>` — Rotates the Vicon world onto the workspace datum, radians about
  down. This is where the datum lives under `vicon:=true`, not `frame_yaw_offset` (default: `0.0`).
- `stamp_source:=<header|receipt>` — `header` needs chrony between the laptop and the Pi;
  `receipt` carries the transport latency in `EKF2_EV_DELAY` instead (default: `header`).

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

Every process runs with `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`. A 2.24 MB frame does not fit Fast DDS's default 512 KiB shared-memory segment, and under load it is dropped with no error.

Before flying:

- Load `params/hardware.params` onto the flight controller and resolve its TODOs (declination,
  `MPC_THR_HOVER`, the TELEM2 wiring, the RC mode/kill/consent channels). It is **not** derived from the
  SITL airframe, which a real flight controller cannot load and which disables failsafes.
- Brief the pilot: after granting consent, **let go of the sticks**. Any stick movement in OFFBOARD
  hands the aircraft back (`MAN_OVERRIDE_SPD`), and indoors PX4 does not return to OFFBOARD.
- Brief the pilot again for dropouts: on a lock loss PX4 falls back to Altitude and then **often hands
  OFFBOARD back by itself**. A stick input at that moment ends the run permanently, so taking over is a
  deliberate decision, not a reflex. In SITL, doing nothing for the first second drifts ~0.1 m against
  ~0.9 m when the pilot corrects immediately — in Altitude a stick input is a tilt command.
  **`nav_state` reading Altitude does not mean the loop has lost the aircraft.** During the fallback
  `nav_state_user_intention` stays at 14 (OFFBOARD) — that is PX4 saying it means to give it back, and
  it is the window in which an override does the damage. `px4_20260918_084714` lost a 41 s run this
  way at t=7.1 s, 10 ms after PX4 had already recovered OFFBOARD on its own; the remaining 34 s were a
  failsafed aircraft nobody was flying. The rule is: hands off until `nav_state_user_intention` itself
  leaves OFFBOARD, or until you have decided to end the run.
- Armed in Altitude on the ground, check `/fmu/out/vehicle_status_v4`: if `nav_state_user_intention`
  reads POSCTL, stop — the loop will never get the aircraft, and nothing in it will say so.
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

## Vicon: aiding EKF2 without putting it in the loop

Indoors EKF2 has no horizontal aiding, dead-reckons to kilometres, and PX4 therefore parks the
user-intended mode at POSCTL and never returns the aircraft to OFFBOARD. Vicon fixes the estimator
itself. It is fed **only while the loop does not have the aircraft**: `vicon_px4_bridge` stops
publishing the moment `px4_offboard_bridge` asks for OFFBOARD, and EKF2 drops external vision
400 ms later (`2 x EV_MAX_INTERVAL`). The scored window is unaided, so no ground truth is inside
the loop under test.

Two machines, because the Vicon SDK is **x86-64 only** and cannot run on the Pi:

```
Vicon Tracker --DataStream, wired--> LAPTOP: vicon_client   --lab WiFi-->   PI: vicon_px4_bridge
                                     (ros2-vicon-jazzy)                     -> /fmu/in/vehicle_visual_odometry
                                                                            -> MicroXRCEAgent -> PX4
```

On the laptop, with the **same transport profile the Pi uses** or nothing arrives:

```sh
FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA   ros2 run vicon_receiver vicon_client --ros-args -p hostname:=<vicon host> -p port:=801
```

On the Pi:

```sh
ros2 launch quad_px4 hardware.launch.py vicon:=true consent_rc_aux:=1     quad_mass:=<weighed> hover_thrust:=<measured>
```

Before the first Vicon flight:

- Load the `EKF2_EV_*` block from `params/hardware.params` and **reboot the flight controller** —
  `EKF2_EV_DELAY` does not take effect otherwise. `EKF2_EV_CTRL` is `1`, horizontal position only:
  height stays on the barometer so the cut cannot remove the height reference, and yaw is not
  fused so EKF2's mag heading stays an independent witness for the frame check below.
- Run **chrony** between the laptop and the Pi. Without it the bridge logs a clock error and the
  sample age is meaningless; use `stamp_source:=receipt` if you cannot.
- Set `EKF2_EV_DELAY` from the bridge's own `EV stream: ... age mean` line, then reboot again.
- Measure `EKF2_EV_POS_X/Y/Z`: the Vicon marker centroid relative to the CoG, body FRD.
- Confirm the frames by hand before arming — carry the airframe over the plate and check N/E/D
  signs and heading in `/fmu/out/vehicle_odometry`. `vicon_frame` and `body_frame` are assumptions
  until this is done, and a wrong `body_frame` is a sign error in pitch and yaw that flies.
  The bridge now checks this itself: it **refuses to publish** until the converted Vicon attitude
  agrees with EKF2's to within 30°, and the FATAL names the likely wrong knob. Trust the check to
  catch a gross rotation; still do the hand check, because it is what sets `vicon_yaw_offset`.
- Check `/fmu/out/estimator_status_flags`: `cs_ev_pos` must be true (`cs_ev_yaw` stays false by
  design) and `xy_valid` must become true. A published stream that is not fused looks identical to
  a working one from the ROS side — and a stream in the *wrong frame* looks identical to a correct
  one from inside EKF2, which is what the frame check exists for.

**A new failure mode to brief the pilot on:** a WiFi stall longer than 400 ms drops EKF2 aiding
mid-flight, with no indication other than a PX4 log line. Outside OFFBOARD that means any
position-holding mode falls back under their hands. The bridge stops publishing rather than
repeating a stale pose, deliberately — a frozen mocap sample reads to EKF2 as a perfect
measurement and fights the IMU.

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
