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

## Indoor Vicon lab — no GPS, safety pilot

`venue:=indoor` switches the whole venue at once, so the pieces cannot disagree: EKF2 loses GNSS
aiding and takes height from the barometer, `px4_offboard_bridge` stops arming and waits for the
pilot, `px4_takeoff_gate` stops requiring GNSS, `ibvs_gate` stops requiring a target topic, and
the offboard-loss failsafe becomes Altitude rather than Hold.

```sh
ros2 launch quad_px4 sitl.launch.py venue:=indoor
```

In SITL the pilot is `sim_pilot`, which arms, selects Altitude, climbs, and then **keeps flying** —
an IMU/mag/baro-only aircraft has no horizontal hold at all, so a centred stick departs
(`experiments/e49.md`). It reads `/quad_state` to do that, which models the real pilot's eyes; the
IBVS chain still takes no position anywhere.

**One parameter differs from hardware.** SITL sets `COM_RC_IN_MODE` 1 (MAVLink only) so PX4
accepts `sim_pilot`'s stream over DDS. On the real aircraft it must be **0 (RC only)** — the
safety pilot is on a transmitter.

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
