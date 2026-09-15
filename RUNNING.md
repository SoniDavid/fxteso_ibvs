# Running the FxTESO-IBVS project

The same vision and control code runs in two places: against **PX4 SITL** on the dev desktop, and
on the **companion computer** (Raspberry Pi 5) of the real UAV. Simulation and hardware differ only
in their launch file: `sitl.launch.py` or `hardware.launch.py` and in what feeds the camera topic. This file covers what both machines share; for the rest see:

- [**RUNNING_SIM.md**](RUNNING_SIM.md) — dev desktop, PX4 SITL, disturbances, the observer alone.
- [**RUNNING_HW.md**](RUNNING_HW.md) — the Raspberry Pi, the real UAV, bench testing.

## Setup the environment

Both machines need:
- ROS2 Jazzy Jalisco
- OpenCV ≥ 4.7 and `cv_bridge` built against it (Noble's apt OpenCV 4.6 is too old for
  `image_features`' ArUco detectors)

The desktop additionally needs Gazebo Sim 8.11 - Harmonic and the PX4 Autopilot fork; see
[RUNNING_SIM.md](RUNNING_SIM.md#dev-desktop-sitl). 
The Pi additionally needs the camera stack and MicroXRCEAgent; see [RUNNING_HW.md](RUNNING_HW.md#companion-computer-raspberry-pi-5-ubuntu-2404).

### OpenCV ≥ 4.7 and cv_bridge from source (both machines)

`image_features` detects markers with `aruco_nano` (a header in `quad_control`) or OpenCV's
`cv::aruco::ArucoDetector`; both need the OpenCV 4.7+ API. OpenCV is rebuilt system-wide, not
vendored;same convention as `tools/setup_pi5_ubuntu_camera.sh`.

```sh
# --recurse-submodules matters: without it 3rdparty/quirc is missing and configure fails
git clone --branch 4.14.0 --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/opencv/opencv.git
git clone --branch 4.14.0 --depth 1 https://github.com/opencv/opencv_contrib.git

# Raspberry Pi 5 (Cortex-A76): the dispatch targets apt's generic aarch64 build lacks
CPU_FLAGS="-D CPU_BASELINE=NEON,FP16 -D CPU_DISPATCH=NEON_DOTPROD,NEON_FP16,NEON_BF16 -D WITH_QT=5 -D WITH_GTK=OFF"
# Dev desktop: OpenCV's own x86 defaults
CPU_FLAGS=""

cmake -S opencv -B build -G Ninja \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D CMAKE_INSTALL_LIBDIR=lib/$(gcc -print-multiarch) \
    -D OPENCV_EXTRA_MODULES_PATH="$(pwd)/opencv_contrib/modules" \
    -D WITH_TBB=ON \
    -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF \
    -D BUILD_opencv_python2=OFF -D BUILD_opencv_python3=OFF \
    $CPU_FLAGS

cmake --build build -j$(nproc)          # ~25-30 min on the Pi, real thermal load
sudo cmake --install build && sudo ldconfig
```

`CMAKE_INSTALL_LIBDIR=lib/<multiarch>` is load-bearing: `/etc/ld.so.conf.d/<multiarch>.conf` gives
shadow priority only to `/usr/local/lib/<multiarch>`, not to plain `/usr/local/lib`. Check the
shadow took before trusting anything built against it — the `/usr/local` copy must be listed first:

```sh
ldconfig -p | grep libopencv_core
```

`cv_bridge` must then be rebuilt from source against it. apt's prebuilt `cv_bridge` links the old
SONAME, and a process using both loads **two** OpenCV versions at once. 4.1.0 matches the installed
`ros-jazzy-cv-bridge`, so it is a pure recompile. `src/vision_opencv` is gitignored.

```sh
cd ~/[path_to_project]/fxteso_ibvs/src
git clone --branch 4.1.0 --depth 1 https://github.com/ros-perception/vision_opencv.git
cd .. && colcon build --packages-select cv_bridge image_geometry --allow-overriding cv_bridge image_geometry
```

After any OpenCV version bump, `ldd` every OpenCV-linking binary in the workspace, not just the
one being worked on. Both SONAMEs in one binary is the bug:

```sh
ldd install/quad_control/lib/quad_control/image_features | grep libopencv_core   # .so.414 only, never .so.406
```

## Launch Arguments Reference

You can configure the runs by appending these arguments to the `ros2 launch` commands. Below is an
exhaustive list of all arguments available in `sitl.launch.py`, which `hardware.launch.py` and
`bench.launch.py` both build on — their own additional arguments are documented in
[RUNNING_HW.md](RUNNING_HW.md#launch-arguments-reference).

### Display & Recording
- `headless:=<bool>` — Drop the Gazebo window (default: `false`).
- `rosbag:=<bool>` — Record a bag of the run (default: `false`; `true` in `hardware.launch.py`).
- `foxglove:=<bool>` — Open Foxglove to watch the run over ROS — shows what the aircraft sees (default: `false`).
- `record_from:=<handover|launch>` — When to start recording the rosbag (default: `handover`).

### Environment & Venue
- `venue:=<indoor|outdoor>` — `indoor` disables GNSS and waits for a safety pilot; `outdoor` waits for GNSS heading alignment (default: `indoor`).
- `disturbance:=<none|step|gust|wind|table52|csv>` — Wind and disturbance model (default: `none`).
- `disturbance_seed:=<int>` — Seed for disturbances (default: `0`).
- `gust_scale:=<float>` — Gust magnitude multiplier (default: `1.0`).
- `wind_scale:=<float>` — Steady wind multiplier (default: `1.0`).
- `gust_tau:=<float>` — Gust time constant (default: `1.5`).
- `turbulence_scale:=<float>` — Turbulence multiplier for `table52` (default: `1.0`).
- `wind_velocity:="[x, y, z]"` — Wind direction in m/s (default: `[0.8, 0.4, 0.0]`).
- `mag_acclim:=<float>` — EKF2 threshold for fusing mag heading (default: `0.0`).

### Target Simulation
- `target_profile:=<hover|line|thesis>` — Target trajectory (default: `hover`).
- `target_speed:=<float>` — Target speed (default: `1.0`).
- `target_yaw_rate:=<float>` — Target yaw rate (default: `0.1`).
- `target_accel:=<float>` — Target acceleration (default: `0.5`).
- `target_heading:=<float>` — Target direction of travel in degrees (default: `0.0`).
- `target_scale:=<float>` — Size of the target; `0.5` is the printed 450 × 375 mm plate (default: `0.5`).
- `marker_dict:=<str>` — ArUco marker dictionary (default: `7x7`).
- `blackout_at:=<float>` — Time in seconds to hide the target (default: `0.0`).
- `blackout_for:=<float>` — Duration in seconds to hide the target (default: `3.0`).

### Vision
- `detector_backend:=<nano|opencv|hybrid|roi|nano_roi>` — `nano` is `aruco_nano`; `opencv` is `cv::aruco::ArucoDetector`; `hybrid` runs nano while the previous frame held the full target and falls back to OpenCV on a miss; while unlocked it runs OpenCV alone; `roi` / `nano_roi` search a crop around the last full detection first (corners in full-frame coordinates, so the measurements are identical). In SITL `hybrid` matched OpenCV's reliability at ~40% of its detect time (default: `hybrid`).
- `nano_error_correction:=<float>`, `nano_border_error_rate:=<float>` — aruco_nano's bit and border error tolerance. Its own defaults (`0.0`, `0.0`) reject a marker for a single mis-read bit, the main cause of nano losing lock under motion; `0.3` / `0.35` restore OpenCV's recall at ~0.3× its cost (defaults: `0.3`, `0.35`).
- `nano_box_filter:=<int>`, `nano_max_revisited:=<float>` — aruco_nano's threshold window and contour tracer (defaults: `15`, `0.05`).
- `roi_margin:=<float>` — `roi` backends: margin around the last target box, as a fraction of its size (default: `0.5`).
- `use_aruco3_detection:=<bool>` — `opencv` only: search marker candidates on a downscaled image (default: `false`). **Broken at this geometry:** with the derived ratio it decoded no markers on OpenCV 4.6 or 4.14 (0/3 locked in SITL), and decoding is gone once the ratio passes ~0.015. Leave it off.
- `aruco3_margin:=<float>` — Fraction of the nominal marker size aruco3 must still find; the ratio itself is derived from `camera`, `target_scale` and `zD` (default: `0.7`).
- `cv_num_threads:=<int>` — OpenCV thread count, `0` leaves it alone. At `0` OpenCV's idle pool burns ~0.9 cores next to the hybrid; `1` removes that, but single-threaded fallback frames are slower (worst ~19 ms of the 20 ms budget in SITL), so check the Pi's worst frame with `bench.sh --cv-threads 1` and `2` (default: `1`).
- `image_features_cpu:=<core>` — Pin `image_features` to a core, empty leaves it unpinned (default: empty; hardware and bench only).

### Flight Geometry & Observer
- `zD:=<float>` — Servoing depth (default: `1.2`).
- `takeoff_alt:=<float|zD>` — Takeoff altitude. Pass literal `zD` to launch directly at the servoing depth (default: `1.5`).
- `takeoff_tolerance:=<float>` — Required proximity to `takeoff_alt` before gate opens (default: `0.10`; `0.20` on hardware, barometer-only height).
- `eso_z_des:=<float>` — Observer's desired servoing depth (defaults to tracking `zD`).
- `observer_omega:=<float>` — If > 0, sets fixed_eso's x/y gains as a triple pole at this rate (default: `0.0`).
- `gamma1_xy`...`gamma4_yaw` — Various observer gains. Defaults: `gamma1_xy=18.0`, `gamma2_xy=20.0`, `gamma3_xy=4.0`, `gamma1_yaw=5.0`, `gamma2_yaw=16.0`, `gamma3_yaw=3.0`, `gamma4_yaw=0.001`, `alpha_yaw=0.75`, `beta_yaw=1.2`.
- `eso_yaw_sign:=<float>` — Yaw sign for the observer (default: `1.0`).
- `initial_estimate_offset:="[<qx>, <qy>, <qz>, <qpsi>]"` — Initial estimate error offset (default: `[0.0, 0.0, 0.0, 0.0]`).
- `estimators_ready:=<float>` — Settled hover time required before handover (default: `5.0`).
- `max_feature_error:=<float>` — Required marker alignment before pos_ctrl takes over (default: `0.15`).
- `gate_timeout:=<float>` — Timeout for the gate (default: `120.0`).
- `controllers:=<bool>` — Enable or disable controllers (default: `true`).

### Plant Configurations
- `plant:=<analytic|gazebo>` — Plant model used in simulation (for `sim.launch.py`).
- `camera:=<preset>` — Camera config preset (e.g., `module2_1640`, `module3wide_2304`).
- `camera_rate:=<float>` — Camera rate (default: `0.0`).
- `prop:=<str>` — Rotor model (default: `9545`).
- `battery_cells:=<int>` — Battery cell count (default: `4`).
- `hover_thrust:=<float>` — Measured `MPC_THR_HOVER` (default: `0.6461`).
- `offboard_recovery:=<bool>` — Attempt offboard recovery on lock-loss (default: `false`).
- `px4_dir:=<path>` — Path to the PX4 fork submodule.
- `xrce_agent:=<path>` — Path to the MicroXRCEAgent binary.

## Camera Presets

A `camera:=` preset is a Raspberry Pi module **and a sensor mode**, from
`quad_description/config/cameras.yaml`. Not every module reaches the 50 Hz `image_features` loop
at full field of view. In simulation one preset rewrites the `<camera>` block in the SDF **and**
feeds `image_features` the same intrinsics. On the UAV the same preset sizes the picamera2 driver's
output **and** `image_features`. Either way, what is seen and what is modelled cannot drift apart.
`zD:=` likewise drives `aD` and PX4's `MIS_TAKEOFF_ALT` together:

```
aD = 0.5625 * target_scale^2 * (0.00304 / zD)^2
```

It is `aD`, not `zD`, that decides where the aircraft settles.

**`sitl.launch.py` and `hardware.launch.py` defaults**: `module3wide_2304` (1152 × 648), takeoff
1.5 m, `zD` 1.2, `target_scale` 0.5, `detector_backend` hybrid. A bare SITL launch takes off,
descends onto a stationary target and holds.

**`sim.launch.py` / `observer_only.launch.py` defaults**: `zD` 2.5, `target_scale` 1.0 — neither
plant takes off so `takeoff_alt` is unused.
    quad_mass:=<weighed, battery in> hover_thrust:=<measured MPC_THR_HOVER \ consent_rc_aux:=1

