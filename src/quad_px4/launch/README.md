# `sitl.launch.py` — running the PX4 SITL stack

FxTESO + ANFTIBVS flown against PX4 SITL as the plant. The module docstring at the top of
`sitl.launch.py` has the node graph and the start-up sequence; this file is the argument
reference and the recipes.

    ros2 launch quad_px4 sitl.launch.py [arg:=value ...]

## Before the first run

Two one-off steps, plus the overlays every run needs.

    git submodule update --init --recursive external/PX4-Autopilot
    make -C external/PX4-Autopilot px4_sitl_default

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_jazzy/install/setup.bash
    source ~/Robotics/fxteso_ibvs/install/setup.bash

All three overlays, in that order — with fewer, `ros_gz_sim` is missing and bag
deserialization fails.

Check nothing is already flying before you launch. An orphaned `gz sim` holds a GPU context
and starves PX4's magnetometer, and every later run then fails on `mag-stale`:

    pgrep -af "g[z] sim|px[4]_sitl|MicroXRCE"

The launch file reaps orphans whose parent is pid 1, but not a simulation you left running
in another terminal.

## Quick start

    # watch it fly: calm air, stationary target
    ros2 launch quad_px4 sitl.launch.py

    # the same, recorded and without the Gazebo window
    ros2 launch quad_px4 sitl.launch.py headless:=true rosbag:=true

`RUNS.md` at the repository root lists the standing configurations. Run `./cleanup.sh` before
each one - it kills a previous run's processes and waits for the ports to be released.

## Arguments

### Session

| argument | default | what it does |
| --- | --- | --- |
| `headless` | `false` | `true` drops the Gazebo GUI. Sensor rendering still runs, and RTF is no longer pinned to 1.0. |
| `controllers` | `true` | `false` skips `control.launch.py` after the gates — the aircraft takes off and loiters. Use it to check takeoff, camera framing and lock without handover. |
| `rosbag` | `false` | `true` records the topic set in `quad_utils/config/bag_topics.yaml` to `bags/px4_<timestamp>`, mcap, with `--use-sim-time`. Recording starts at handover, not at launch. |
| `record_from` | `handover` | Where the recorder starts. `handover` is what every flight wants. `launch` is for measuring the **observer**: `fixed_eso` starts at the takeoff gate and converges in ~1 s, so a recorder started at handover — or even at the gate, which needs ~1 s to come up — misses the transient entirely. |
| `foxglove` | `false` | `true` starts the Foxglove bridge alongside the TF/marker visualisation, and logs the path of the Studio layout to import. There are two variants: the Vision tab follows whichever image topic `image_features` consumes, which depends on the camera preset. Import the one the launch names. |
| `px4_dir` | `~/Robotics/fxteso_ibvs/external/PX4-Autopilot` | The PX4 fork holding `build/px4_sitl_default`. Launch fails loudly if the binary is missing. |
| `xrce_agent` | `~/Robotics/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent` | The uXRCE-DDS agent binary. It is not normally on `PATH`. |

### Camera and target geometry

`camera` selects a Raspberry Pi module **and a sensor mode** from
`quad_gz_sim/config/cameras.yaml` — one table that feeds both the `<camera>` block gz-sim
renders and `image_features`' feature model, so the two cannot disagree. Modes are separate
presets because not every module reaches the 50 Hz loop at full field of view.

| preset | hfov | render | fps | note |
| --- | --- | --- | --- | --- |
| `module2_1640` | 62.2° | 820×616 | 41.9 | Camera Module 2, the narrower lens. Below the 50 Hz loop. |
| `module2_1640_native` | 62.2° | 1640×1232 | 41.9 | Same optics at native resolution — the one preset that breaks the half-resolution rule. |
| `module2_1640_8bit` | 62.2° | 820×616 | 83.7 | 8-bit readout; the only way IMX219 clears 50 Hz without losing FOV. |
| `module2_1920` | 38.9° | 960×540 | 47.6 | Cropped mode. Kept to make the FOV cost measurable, not because it is usable. |
| `module3_2304` | 66.0° | 1152×648 | 56.0 | Module 3. Wider horizontally, **narrower vertically** — and the short axis binds. A downgrade here. |
| `module3wide_2304` | 102.0° | 1152×648 | 56.0 | **default.** Module 3 Wide. Full FOV above 50 Hz, which is what makes the acquisition transient fit. Barrel coefficients are an assumption, not a calibration. |
| `module3wide_2304_ideal` | 102.0° | 1152×648 | 56.0 | The same with distortion zeroed. Not a real camera — the control arm separating FOV from lens. |
| `module3wide_1536` | 78.9° | 768×432 | 120.1 | High-rate fallback, cropped. Its coefficients overstate edge distortion at this crop. |

A preset with non-zero distortion pulls in the `camera_distort` node automatically and
repoints `image_features` at `/quad/camera/image_distorted`; a zero vector leaves the path on
`/quad/camera/image_raw`. Nothing to pass either way.

| argument | default | what it does |
| --- | --- | --- |
| `camera` | `module3wide_2304` | Preset from the table above. An unknown name fails at launch with the valid list. |
| `target_scale` | `0.5` | Scales the ArUco target; the default is 450 × 375 mm printed. Larger markers buy field-of-view slack and cost decode pixels. `aD` follows automatically. |
| `marker_dict` | `7x7` | `7x7` (the thesis, and every archived bag) or `4x4`, which decodes at about two thirds the pixel size. Same IDs, so corner ordering is unchanged. |
| `camera_rate` | `0.0` | Above zero, overrides the sensor's update rate — use it to fly the real mode's fps against the 50 Hz loop instead of the sim's free 50. |
| `zD` | `1.2` | Servoing depth. `aD`, `MIS_TAKEOFF_ALT` and the takeoff gate are all derived from it, so it is one number, not three. See `RUNS.md` for the 2.5 m configuration. |

### Disturbance

Magnitudes live in `quad_gz_sim/config/disturbances.yaml`; the scale arguments multiply them
so a sweep varies one number while the YAML keeps owning the shape. `/disturbances` is an
inertial-frame force in newtons on a 2 kg airframe, held until `pos_ctrl` publishes.

| argument | default | what it does |
| --- | --- | --- |
| `disturbance` | `none` | `none`, `step`, `gust`, `wind`, `table52`, `csv`. Profiles **compose**: `disturbance:=wind,step`. |
| `disturbance_seed` | `0` | RNG seed for `gust` and for `wind`'s turbulence. Same seed, same realisation. |
| `gust_scale` | `1.0` | Multiplies `gust_sigma` (0.10, 0.10, 0.05 N). |
| `wind_scale` | `1.0` | Multiplies `wind_velocity` (0.8, 0.4, 0.0 m/s) **and its turbulence**. Drag is quadratic, so quote realised force, never the scale. |
| `gust_tau` | `1.5` | The gust's correlation time in seconds. Sweep it against the estimator lag. |
| `turbulence_scale` | `1.0` | `table52` only. Scales the Von Karman sigmas and nothing else, so `0.0` flies the thesis' wind **structure** — the reversals and the downdraft — with no gusting. That separates what the schedule costs from what the turbulence costs. |

`table52` is the thesis' own Table 5.2: a mean schedule that reverses direction, carries a
−4 m/s downdraft, and is switched off at t > 190 s so convergence can be shown afterwards. The
schedule and the turbulence intensities live in `disturbances.cpp`, not in the YAML — they are a
specification to reproduce, and a run with edited intervals is not Table 5.2. The altitude the
altitude-dependent sigmas are evaluated at follows `zD` automatically.

### Target trajectory

| argument | default | what it does |
| --- | --- | --- |
| `target_profile` | `hover` | `thesis`, `hover`, `line`, `circle`, `steps`. `thesis` is the archived trajectory: it ends near t = 165 s with hover to ~260 s, and its 0.9 m/s peak wants the 2.5 m geometry — see `RUNS.md`. |
| `target_speed` | `1.0` | m/s. Used by `line`, `circle`, `steps`. |
| `target_yaw_rate` | `0.1` | rad/s. Used by `circle`. |
| `target_accel` | `0.5` | m/s². Used by `line` and `steps`. |

`thesis` and `hover` ignore all three. Under this plant the target is held on its start pose
until `pos_ctrl` publishes — arming and takeoff cost sim time the other plants do not spend,
and without the hold the target leaves the camera footprint before lock.

### Gates and handover

Start-up runs through `px4_takeoff_gate` (settled at the servoing altitude → start the
estimators) then `ibvs_gate` (plant, target and a held marker lock → start `pos_ctrl`).
`pos_ctrl` publishing `desired_attitude` is what tips `px4_offboard_bridge` into OFFBOARD.

| argument | default | what it does |
| --- | --- | --- |
| `takeoff_alt` | `1.5` | Depth at handover; empty means "use `zD`". 1.5 against `zD` 1.2 is the deployment profile — launch high and descend onto the target rather than climb to it. Setting it away from `zD` makes the initial depth error a swept variable rather than an accident. One resolver feeds PX4, the bridge and the gate, so they cannot disagree. |
| `takeoff_tolerance` | `0.10` | How close to `takeoff_alt` the gate insists on. The node's own 0.5 m default straddles the depth stability threshold. |
| `estimators_ready` | `5.0` | Seconds of settled hover held before handover. EKF2's pitch reads ~0.019 rad high early; commanding zero against that is uncommanded forward acceleration into the tight FOV axis. |
| `max_feature_error` | `0.15` | Marker alignment required before handover. `\|qpsi\|` at handover separates held from collapsed runs at ~0.17; `0` disables the test. |

Both gates carry wall-clock timeouts — 180 s on takeoff, 120 s on lock (`gate_timeout`). A gate
that times out logs why, the stage after it never starts, and the launch tears itself down
rather than idling: EKF2 never recovers from a failed initialisation, so the remaining minutes
buy nothing and in a sweep they are the single largest cost.

`px4_takeoff_gate` carries two further deadlines, both launch arguments on the node:

| argument | default | what it does |
| --- | --- | --- |
| `aiding_deadline` | `25.0` | Seconds after EKF2 **starts publishing** before missing aiding is called terminal. Tests **both** `cs_yaw_align` and `cs_gnss_pos` — separate failures with one cost, since `px4_offboard_bridge` arms on `xy_valid && z_valid` and `xy_valid` needs horizontal aiding. Measured from the first `estimator_status_flags`, not from the node's start, so it does not also count PX4's 9–10 s of boot and vary with machine speed. 25 s covers the slowest legitimate start: `cs_tilt_align` has been seen at 7.2 s and `checkMagField()` then refuses for a further mandatory second, while a healthy start has both flags up within ~1 s. `0` disables. |
| `ekf_silence_deadline` | `60.0` | Backstop for the other failure — PX4 never comes up at all, so no EKF2 message ever arrives and `aiding_deadline` never starts counting. |

**If `aiding_deadline` starts firing, read the ulog before raising it.** ~55% of starts once
failed to align yaw, and it was never a race: `F450_base/model.sdf` gave the magnetometer no
`<stddev>`, so its output was byte-identical while the aircraft sat still, PX4's `DataValidator`
counted 1000 equal values and stopped publishing `vehicle_magnetometer` altogether — and EKF2
lost its only heading source before it could arm and move. Fixed at the model, 10/10 starts
align since. A sensor that goes quiet mid-run means something has lost its noise again.

### PX4 plant

| argument | default | what it does |
| --- | --- | --- |
| `prop` | `9545` | Rotor model. Only the 9545 is benched; nothing else is in the table. |
| `battery_cells` | `4` | 4S, not 3S: at 2.0 kg a 9545 on 3S needs 91% throttle to hover, leaving the attitude loop nothing. 4S gives T/W 2.07. |
| `hover_thrust` | `0.6461` | Anchors the newton → normalised thrust map in `px4_offboard_bridge`. It is cross-checked against the value derived from `prop` and `battery_cells`, and launch **fails** if they differ by more than 0.005 — a silent drift here mis-scales every command. |

`prop` and `battery_cells` also set `SIM_GZ_EC_MAX*` and `MPC_THR_HOVER` from one derivation,
and the launch file forces `COM_OBL_RC_ACT=5` (Hold): the default 0 = Position expects RC that
SITL does not have, so losing marker lock would make the aircraft descend.

### Observer and controller gains

The control law is the object of study — these exist to sweep it, not to fix it. Defaults are
the flown set; Table 5.3 has `gamma2_xy` 10, `gamma3_xy` 7, `gamma3_yaw` 7.

| argument | default | | argument | default |
| --- | --- | --- | --- | --- |
| `gamma1_xy` | `18.0` | | `gamma1_yaw` | `5.0` |
| `gamma2_xy` | `20.0` | | `gamma2_yaw` | `16.0` |
| `gamma3_xy` | `4.0` | | `gamma3_yaw` | `3.0` |
| `alpha_yaw` | `0.75` | | `beta_yaw` | `1.2` |
| `gamma4_yaw` | `0.001` | | `eso_yaw_sign` | `1.0` |

`observer_omega` (default `0.0`) is the alternative to `gamma1_xy`: above zero it places
`fixed_eso`'s x/y gains as a triple pole at that rate and overrides `gamma1_xy`. Zero keeps
the flown set.

`eso_yaw_sign` is the observer's `g(xi)*u` sign on yaw — `1.0` is as-flown, `-1.0` is thesis
Eq. 5.81's −1.

`att_ctrl` does not run under this launch file at all. PX4 owns allocation, attitude and
rates; everything from `image_features` through `pos_ctrl` is the same code the analytic and
gazebo plants use.

`initial_estimate_offset` (default `[0.0, 0.0, 0.0, 0.0]`) seeds `fixed_eso`'s initial state
with an estimation error `(qx,qy,qz,qpsi)`. Sweeping it is how the fixed-time claim gets
measured — settling time must stay bounded as it grows — and it needs `record_from:=launch` to
be visible at all. Note that the observer **peaks** before it converges, so the measured `|e₀|`
is larger than the seeded offset and is what the result must be keyed on.

## Recipes

The standing configurations — the deployment and thesis conditions, the wind cases, the
observer ladder — are in `RUNS.md` at the repository root. These are the argument shapes it
does not cover:

    # separate what the field of view buys from what the assumed barrel profile costs
    ros2 launch quad_px4 sitl.launch.py camera:=module3wide_2304_ideal

    # gust, seeded so the realisation repeats
    ros2 launch quad_px4 sitl.launch.py disturbance:=gust disturbance_seed:=7 gust_scale:=2.0

    # steady wind plus a pulse, composed
    ros2 launch quad_px4 sitl.launch.py disturbance:=wind,step wind_scale:=1.5

    # takeoff and camera framing only, no handover
    ros2 launch quad_px4 sitl.launch.py controllers:=false

    # no initial depth error at handover — takeoff_alt defaults to 1.5 against zD 1.2
    ros2 launch quad_px4 sitl.launch.py takeoff_alt:=1.2

    # place the observer's x/y gains as a triple pole instead of setting gamma1_xy
    ros2 launch quad_px4 sitl.launch.py observer_omega:=6.0

## Reading the log

- A one-line camera summary prints before the estimators start. It names the preset and warns
  if the mode's fps is **below** the 50 Hz loop — the cheapest confirmation `camera:=` took.
- `px4_takeoff_gate` then `ibvs_gate` must both exit 0. Either failing logs why, and the stage
  after it is skipped rather than run blind.
- `controllers:=false` logs that the aircraft will loiter, so a loiter is never mistaken for a
  servoing run.

Sanity checks from another sourced shell:

    ros2 topic hz /quad/camera/image_distorted     # or image_raw, ~50 Hz
    ros2 topic echo /quad/camera/camera_info --once
    ros2 topic echo /disturbances --once

Read yaw from `qpsi` or `/quad_state`, never `/quad_attitude`: EKF2 carries a ~6.5° heading
bias here, benign for control and poison for any yaw metric.
