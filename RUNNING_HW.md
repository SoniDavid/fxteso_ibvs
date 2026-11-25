# Running on hardware

The Raspberry Pi 5 runs the vision and control stack and flies a PX4 quadrotor over a printed ArUco marker. Vicon aids PX4's estimator only; it never enters the controller. Shared setup: [RUNNING.md](RUNNING.md). Simulation: [RUNNING_SIM.md](RUNNING_SIM.md).

> Flown on an F450 (3S, 1.30 kg) with a Pixhawk 6C Mini, a Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy), a Camera Module 3 Wide and one 7x7 marker, indoors under Vicon.

## What runs where

| Machine | Runs | Started with |
|---|---|---|
| Raspberry Pi 5 | camera, image features, FxTESO, controller, PX4 bridges, bag | `ros2 launch quad_px4 flight.launch.py ...` |
| Desktop (x86-64) | Vicon receiver; depth commands | `ros2 launch quad_px4 vicon_receiver.launch.py`; `ros2 run quad_px4 zd_set.py <m>` |
| Flight controller | PX4: EKF2, attitude loop, failsafes | `params/hardware.params`, then `params/hardware_vicon.params` |

Every node runs with `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`; the launch files and `zd_set.py` set it themselves. A plain shell needs `export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` to see the Pi's topics.

## 1. Setup

### 1.1 Pi (once)

```sh
bash tools/setup_pi5_ubuntu_camera.sh                  # libcamera fork + picamera2, 20-45 min
python3 tools/camera_module3_wide_test.py --doctor     # camera check
# OpenCV + cv_bridge from source: RUNNING.md, with the Pi's CPU_FLAGS

git submodule update --init external/Micro-XRCE-DDS-Agent src/px4_msgs
cmake -S external/Micro-XRCE-DDS-Agent -B external/Micro-XRCE-DDS-Agent/build
cmake --build external/Micro-XRCE-DDS-Agent/build -j$(nproc)
sudo cmake --install external/Micro-XRCE-DDS-Agent/build && sudo ldconfig

# no simulator, no sim_pilot (it arms the aircraft)
colcon build --symlink-install --packages-ignore quad_gz_sim \
    --allow-overriding cv_bridge image_geometry --cmake-args -DBUILD_SIM_PILOT=OFF

source /opt/ros/jazzy/setup.bash
source ~/[path_to_project]/fxteso_ibvs/install/setup.bash
```

### 1.2 Update the Pi to `main`

```sh
git stash push -u -m "pi local $(date +%F)"            # only if `git status` lists local edits
git switch main && git pull
colcon build --symlink-install --packages-select quad_control quad_px4 quad_utils \
    --packages-ignore quad_gz_sim --cmake-args -DBUILD_SIM_PILOT=OFF
```

Launch files are symlinked: a launch-only change needs no rebuild.

### 1.3 Flight controller and camera (once per airframe)

1. Load `params/hardware.params` and resolve its TODOs (declination, RC mode/kill/consent channels).
2. Load `params/hardware_vicon.params` on top, then reboot.
3. Set `MPC_THR_HOVER` from a real hover (0.665 on the F450).
4. Calibrate the lens: `python3 tools/camera_module3_wide_test.py --calib 20` → `camera_distortion`.
5. Vicon, once per lab: see [First Vicon setup](#first-vicon-setup).

## 2. Fly

### 2.1 Start

Desktop:

```sh
source /opt/ros/jazzy/setup.bash
source ~/[path_to_project]/fxteso_ibvs/install/setup.bash
ros2 launch quad_px4 vicon_receiver.launch.py           # Vicon PC 192.168.1.23:801
```

Pi, in tmux, **with the aircraft on the ground**:

```sh
ros2 launch quad_px4 flight.launch.py vicon_yaw_offset:=-2.156 hover_thrust:=0.665 alpha_xy:=0.0005
# vicon_yaw_offset: re-measure whenever the Vicon object is re-created
```

### 2.2 Before arming

| Startup line | Must show |
|---|---|
| `ANFTSMC alpha_xy` | the gain of position control |
| `Frame check PASSED` | roll and pitch within 1° of Vicon (yaw up to ~35° off is the compass) |
| `Vicon aiding ON` | present |
| `camera mount: image top -> 90` | 90 on the F450 |
| `not in control - holding ...` | the controller is waiting at a level hover |
| `markers/frame`, `decode rate p=1.000` | the marker is in view |

Battery charged to 12.6 V.

### 2.3 Handover

1. Position mode: hover centred over the marker at about 1.3 m (Vicon).
2. Switch ch 7 on. After 3 s of lock: `RC consent ... GRANTED` → OFFBOARD → `in_control TRUE`.
3. Hands off the sticks.
4. Switch ch 7 off to end: PX4 Position takes over and the controller holds again.

### 2.4 Depth steps (desktop, while in control)

```sh
ros2 run quad_px4 zd_set.py 0.8      # prints "accepted" or "REFUSED"; range 0.7-1.6 m
ros2 run quad_px4 zd_set.py 1.2      # wait >= 20 s at each level
```

### 2.5 After the flight

```sh
# Pi: Ctrl-C the launch before powering down, so the bag gets its metadata
scp -r <pi>:<path_to_project>/fxteso_ibvs/bags/flight_<stamp> .
ros2 bag reindex flight_<stamp> -s mcap                 # only if metadata.yaml is empty
```

## 3. Presets

| Launch | For | Sets | You pass |
|---|---|---|---|
| `flight.launch.py` | Vicon flights | Vicon aiding all flight, single marker, ch 7 consent, mass 1.30 kg, no altitude checks, bags `flight_*` | `vicon_yaw_offset`, `hover_thrust` |
| `table_test.launch.py` | props-off table test, no Vicon | single marker, ch 7 consent, no altitude checks, bags `table_*` | `frame_yaw_offset` (minus the EKF2 heading) |
| `hardware.launch.py` | the base both presets include | defaults below | anything |

Both presets expect the flight controller on USB (`/dev/ttyUSB0`). Any argument can still be overridden.

## 4. Pilot and safety

| Situation | Do |
|---|---|
| After consent | Hands off. Any stick input in OFFBOARD hands the aircraft back to PX4. |
| Marker lost | PX4 takes over and holds. Stay hands-off unless you are ending the run. |
| Abort | ch 7 off → Position holds. Use the kill switch only if low or crashing. |
| Vicon or WiFi stall > 0.4 s | EKF2 loses aiding and Position mode can drift; land. |
| Armed, and `nav_state_user_intention` reads POSCTL | Stop: the loop will never get the aircraft. |

## 5. Bench testing (no flight controller)

```sh
./src/quad_cam/scripts/bench.sh --load synthetic --duration 120                       # fake plant and lock: timing only
./src/quad_cam/scripts/bench.sh --load synthetic --att true --duration 120            # + att_ctrl
./src/quad_cam/scripts/bench.sh --load target --layout single_marker --duration 120   # real camera on the printed marker
./src/quad_cam/scripts/bench.sh --load synthetic --backend opencv                     # detector A/B
```

Each loop passes when `achieved >= 0.98 x target`. The report is `bench_<timestamp>.log`.

Camera alone:

```sh
ros2 launch quad_cam camera.launch.py image_qos:=sensor_data
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
ros2 run quad_cam probe_latency --ros-args -p image_qos:=sensor_data      # rate, frame age, jitter
ros2 run quad_cam probe_detect --ros-args -p image_qos:=sensor_data -p target_layout:=single_marker
```

| `bench.launch.py` argument | Default | Meaning |
|---|---|---|
| `load` | `synthetic` | `synthetic` (fake plant), `target` (printed marker), `none` (idle chain) |
| `attitude_controller` | `false` | add `att_ctrl` |
| `image_qos` | `sensor_data` | camera and `image_features` QoS |

## 6. Target

```sh
python3 src/quad_description/scripts/make_markers.py single_marker 7x7 --print
# -> src/quad_description/models/aruco_single_marker/print/single_marker_7x7.pdf
```

| Rule | Why |
|---|---|
| Print at actual size, not fit to page | the PDF is exactly US letter |
| Measure the black square: 176.6 mm for 7x7 | 1% size error = 1% depth error |
| ID 4, dictionary 7x7, white border kept | a second ID 4 in view breaks the lock |
| Flat and matte; align one edge with the heading you want | a square gives yaw only modulo 90° |

## 7. Argument reference (`hardware.launch.py`)

Shared arguments (`zD`, `camera`, target and detector options) are in [RUNNING.md](RUNNING.md#launch-arguments-reference).

**Vehicle and link**

| Argument | Default | Meaning |
|---|---|---|
| `quad_mass` | `2.0` | kg, battery in (`flight.launch.py`: 1.30) |
| `hover_thrust` | `0.60` | the FC's `MPC_THR_HOVER` |
| `zD`, `takeoff_alt` | `1.2`, `1.5` | servo depth and takeoff height, m |
| `takeoff_tolerance`, `handover_tolerance` | `0.20`, `0.15` | altitude bands, m (presets: off) |
| `agent`, `agent_device`, `agent_baud` | `true`, `/dev/ttyAMA0`, `921600` | MicroXRCEAgent to the FC (presets: `/dev/ttyUSB0`) |
| `rosbag`, `bag_prefix` | `true`, `hw` | record a bag under `bags/` |

**Handover and control**

| Argument | Default | Meaning |
|---|---|---|
| `consent_rc_aux` | `0` | RC aux channel that grants the handover (0 = service only) |
| `min_lock_before_handover` | `3.0` | s of continuous lock before handover |
| `ibvs_gate` | `advisory` | `advisory`: gate only reports; `block`: SITL ordering |
| `advisory_feature_error` | `0.5` | alignment limit of the advisory report |
| `hold_until_in_control` | `true` | controller held at level hover until OFFBOARD |
| `alpha_xy` | `6e-6` | x/y adaptive-gain rate (thesis); flown `5e-4` |
| `zD_min`, `zD_max` | `0.7`, `1.6` | bounds for `zd_set.py` targets, m |
| `velocity_source` | `off` | velocity in the controller's Coriolis term; keep `off` |
| `frame_yaw_offset` | `0.0` | workspace datum without Vicon, rad |

**Camera and vision**

| Argument | Default | Meaning |
|---|---|---|
| `camera` | `module3wide_2304` | camera preset (RUNNING.md) |
| `camera_source` | `inprocess` | `image_features` opens the camera; `topic` = external driver |
| `camera_image_top_deg` | `90` | image top on the airframe, deg clockwise from the nose |
| `camera_distortion` | zeros | `[k1, k2, p1, p2, k3]` from a calibration |
| `af_mode`, `lens_position` | `manual`, `0.83` | focus; 0.83 dioptres ≈ 1.2 m |
| `detector_backend` | `nano_roi` | ArUco detector (RUNNING.md) |
| `image_timeout` | `0.2` | s without a frame before the lock is lost |
| `debug_image_hz` | `1.0` | grey frame on `camera_topic` for viewing; 0 = off |
| `target_layout` | `four_markers` | `single_marker` in both presets |
| `marker_dict` | `7x7` | must match the print |
| `camera_cpu` | `2` | CPU core for the camera |

**Vicon**

| Argument | Default | Meaning |
|---|---|---|
| `vicon` | `false` | start `vicon_px4_bridge` (`flight.launch.py`: true) |
| `vicon_topic` | `/vicon/fxteso_f450quad/fxteso_f450quad` | the receiver's pose |
| `aiding_policy` | `gated` | `always` (flight), `gated`, `oneshot`, `manual` |
| `vicon_yaw_offset` | `0.0` | Vicon world onto the workspace datum, rad |
| `vicon_frame`, `body_frame` | `enu`, `flu` | Vicon world and object axes |
| `frame_check_tolerance` | `30.0` | deg, Vicon vs EKF2 before publishing (flight: 60) |
| `vicon_yaw_guard` | `60.0` | deg; a flipped heading is sent as position only |
| `stamp_source` | `header` | `header` needs chrony; `receipt` otherwise |

<a id="first-vicon-setup"></a>
<details>
<summary><b>First Vicon setup</b> (once per lab)</summary>

| Check | How |
|---|---|
| EV parameters active | load the `EKF2_EV_*` block, then reboot (`EKF2_EV_DELAY` needs it) |
| Clocks | chrony between desktop and Pi, or `stamp_source:=receipt` |
| `EKF2_EV_DELAY` | from the bridge's `EV stream: ... age mean` line, then reboot |
| `EKF2_EV_POS_X/Y/Z` | the marker centroid relative to the CoG, body FRD |
| Frames | carry the airframe over the target; check N/E/D and heading in `/fmu/out/vehicle_odometry`; this sets `vicon_yaw_offset` |
| Fusion | `/fmu/out/estimator_status_flags`: `cs_ev_pos`, `cs_ev_hgt`, `cs_ev_yaw` true; `EKF2_SENS_EN` bit 3 set |

</details>
