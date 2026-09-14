# Standing configurations

The launch lines that get used, so they do not have to be reconstructed each time. Copy one
into a terminal; nothing here runs as a script.

Run `./cleanup.sh` before each one — it kills a previous run's processes and waits for the
ports to be released.

Sourcing, in this order. `/opt/ros/jazzy` has to come first or `ros_gz_sim` is missing, and
the ROS setup files reference unbound variables, so do not `set -u` around them.

```sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash
source ~/Robotics/fxteso_ibvs/install/setup.bash
```

Add `rosbag:=true` to record, and `headless:=true foxglove:=true` to drop the Gazebo window
and watch over ROS instead — that view shows what the aircraft sees rather than what the world
looks like.

## Deployment geometry — 1.5 m takeoff, 1.2 m servoing

These are the defaults, so the lines are short.

```sh
# stationary target, calm air
ros2 launch quad_px4 sitl.launch.py

# constant-velocity target
ros2 launch quad_px4 sitl.launch.py target_profile:=line target_speed:=0.7

# steady wind, stationary target
ros2 launch quad_px4 sitl.launch.py disturbance:=wind wind_scale:=3.0

# the scheduled mean wind, no turbulence on it
ros2 launch quad_px4 sitl.launch.py disturbance:=table52 turbulence_scale:=0.0

# the same wind against a moving target
ros2 launch quad_px4 sitl.launch.py disturbance:=table52 turbulence_scale:=0.0 \
    target_profile:=thesis

# the full profile, turbulence included
ros2 launch quad_px4 sitl.launch.py disturbance:=table52
```

## Thesis geometry — 2.5 m

```sh
# calm air
ros2 launch quad_px4 sitl.launch.py zD:=2.5 takeoff_alt:=2.5 target_scale:=1.0 \
    target_profile:=thesis

# on the Module 2 camera
ros2 launch quad_px4 sitl.launch.py zD:=2.5 takeoff_alt:=2.5 target_scale:=1.0 \
    target_profile:=thesis camera:=module2_1640

# with the Table 5.3 observer gains
ros2 launch quad_px4 sitl.launch.py zD:=2.5 takeoff_alt:=2.5 target_scale:=1.0 \
    target_profile:=thesis gamma2_xy:=10.0 gamma3_xy:=7.0 gamma3_yaw:=7.0
```

## Two knobs added 2026-09-05

`eso_z_des` — the observer's desired servoing depth, thesis Eq. 5.81's `z_d`. **It now tracks
`zD` automatically and that is the fix**: it used to be hardcoded at 2.5 m in `fixed_eso.cpp`
regardless of `zD`, so at the 1.2 m deployment depth the observer cancelled only `1.2/2.5 = 48%`
of the commanded acceleration and booked the rest as disturbance, which `pos_ctrl` fed back in.
Correcting it took station-keeping from **0.409 m to 0.0073 m** (`experiments/e51.md`). Pass an
explicit value only to reproduce a pre-fix run:

```sh
# the as-flown behaviour of every run before 2026-09-05, for A/B comparison
ros2 launch quad_px4 sitl.launch.py eso_z_des:=2.5
```

`takeoff_alt:=zD` — take off at the servoing depth instead of descending onto it. Wanted for a
depth SWEEP, where a fixed 1.5 m makes the aircraft *climb* to reach zD 2.5; E38c did that and its
2.5 m arm never acquired. Note `takeoff_alt:=` (empty) is rejected by `ros2 launch` as a malformed
argument even though it means the same thing internally — pass the literal `zD`.

```sh
ros2 launch quad_px4 sitl.launch.py zD:=1.8 takeoff_alt:=zD
```

`fixed_eso` now also logs its depth, mass and gains at startup. It logged nothing before, which is
how the frozen `z_des` survived the whole campaign — and note `run_matrix.py` deletes a successful
run's launch log, so the manifest is the durable record of what was flown.

## The observer on its own

`controllers:=false` so nothing acts on the estimate and what is measured is a property of the
observer alone. `record_from:=launch` because `fixed_eso` starts at the takeoff gate and
converges in about a second — a recorder started at handover misses the transient entirely.

```sh
ros2 launch quad_px4 sitl.launch.py controllers:=false record_from:=launch rosbag:=true \
    initial_estimate_offset:="[0.0, 0.0, -0.5, 0.0]"

# open loop on the analytic plant, no PX4 — much faster to iterate against
ros2 launch quad_utils observer_only.launch.py plant:=analytic \
    initial_estimate_offset:="[0.0, 0.0, -0.5, 0.0]"
```

## Other plants, and visualisation

```sh
ros2 launch quad_utils sim.launch.py plant:=analytic
ros2 launch quad_utils sim.launch.py plant:=gazebo
```

Visualisation against a recorded bag. Pass the camera preset: the layout to import depends on
which image topic `image_features` consumes, and the launch logs the path of the right one.

```sh
ros2 bag play bags/px4_20260826_120000 --clock
ros2 launch quad_utils viz.launch.py foxglove:=true camera:=module3wide_2304
```

## Hardware benchmark — the stack on the Pi, no simulator

`hardware.launch.py` runs the real camera + the estimation/control graph with
`use_sim_time:=false` and nothing simulated (no Gazebo, no PX4 bridge, no `ibvs_gate`).
Each `SimRate` loop logs a `loop:` line every ~10 s (`FXTESO_LOOP_STATS=0` to silence);
`probe_bench` adds achieved rates, per-process CPU, and SoC temp/throttle.

Sourcing on the Pi is just the two lines (there is no `~/ros2_jazzy` / `~/Robotics`
overlay — those are the dev desktop's):

```sh
source /opt/ros/jazzy/setup.bash
source ~/EI-DSoni/fxteso_ibvs/install/setup.bash
```

```sh
# one-shot: launch + probe + write bench_<ts>.log
./src/quad_cam/scripts/bench.sh --load synthetic --duration 120
./src/quad_cam/scripts/bench.sh --load synthetic --att true --duration 120   # + att_ctrl
./src/quad_cam/scripts/bench.sh --load plate --duration 120                   # real ArUco plate

# or by hand
ros2 launch quad_cam hardware.launch.py load:=synthetic
ros2 run quad_cam probe_bench
```

`load:=synthetic` needs no printed target (a `bench_feeder` node fakes the plant, target
and marker lock; `image_features` still runs, its outputs remapped to `*_probe`).
`load:=plate` drives the real vision chain from a printed `{4,6,8,10}` 7×7 plate ~1.2 m
from the lens. `load:=none` leaves `fixed_eso`/`pos_ctrl` idle.

## Rebuilding OpenCV + cv_bridge from source (Pi 5, not vendored)

`image_features`' ArUco `detectMarkers` was the compute ceiling for the 50Hz control loop.
apt's `libopencv-dev` (Noble ships 4.6.0) is a generic-aarch64 build; neither it nor 4.6.0
itself has the CPU dispatch this Pi's Cortex-A76 actually supports (`NEON_DOTPROD`/`FP16`/
`BF16` dispatch targets were added in OpenCV 4.9.0). Same convention as
`setup_pi5_ubuntu_camera.sh` (builds libcamera from source rather than vendoring it): this
isn't committed to the repo, it's rebuilt system-wide and documented here.

```sh
# 1. OpenCV 4.14.0 + opencv_contrib, tuned for this CPU, shadowing apt's copy system-wide.
#    --recurse-submodules matters: without it, 3rdparty/quirc is missing and configure fails.
git clone --branch 4.14.0 --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/opencv/opencv.git
git clone --branch 4.14.0 --depth 1 https://github.com/opencv/opencv_contrib.git

cmake -S opencv -B build -G Ninja \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D CMAKE_INSTALL_LIBDIR=lib/aarch64-linux-gnu \
    -D OPENCV_EXTRA_MODULES_PATH="$(pwd)/opencv_contrib/modules" \
    -D CPU_BASELINE=NEON,FP16 \
    -D CPU_DISPATCH=NEON_DOTPROD,NEON_FP16,NEON_BF16 \
    -D WITH_TBB=ON -D WITH_QT=5 -D WITH_GTK=OFF \
    -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF \
    -D BUILD_opencv_python2=OFF -D BUILD_opencv_python3=OFF

cmake --build build -j$(nproc)          # ~25-30 min, real thermal load
sudo cmake --install build && sudo ldconfig
```

`CMAKE_INSTALL_LIBDIR=lib/aarch64-linux-gnu` is load-bearing, not cosmetic: plain
`CMAKE_INSTALL_LIBDIR=lib` (the default) installs to `/usr/local/lib`, which does **not**
take priority over apt's copy on this system — `/etc/ld.so.conf.d/aarch64-linux-gnu.conf`
(read before `libc.conf` alphabetically) only gives shadow priority to
`/usr/local/lib/aarch64-linux-gnu`. Verify the shadow actually took with
`ldconfig -p | grep libopencv_core` (the `/usr/local/lib/aarch64-linux-gnu` copy should be
listed first) before trusting anything built against it.

```sh
# 2. cv_bridge, rebuilt from source against the new OpenCV - mandatory after any OpenCV
#    version bump, not optional. apt's prebuilt cv_bridge hard-links the OLD version's
#    SONAME; without this, a process using both cv_bridge and the new OpenCV directly
#    loads BOTH versions' .so files at once (confirmed via ldd - a real ABI-collision risk,
#    not hypothetical). 4.1.0 matches the currently-installed ros-jazzy-cv-bridge exactly -
#    this is a pure recompile, not a version change.
cd ~/EI-DSoni/fxteso_ibvs/src
git clone --branch 4.1.0 --depth 1 https://github.com/ros-perception/vision_opencv.git
cd ..
colcon build --packages-select cv_bridge --allow-overriding cv_bridge
```

After any future OpenCV version bump, re-verify with `ldd` on every OpenCV-linking binary
in the graph (not just the one being worked on) - grep its output for old-SONAME entries
(e.g. `libopencv_core.so.406` alongside `.so.414`) to catch this class of bug immediately
rather than as a mysterious runtime issue later.
