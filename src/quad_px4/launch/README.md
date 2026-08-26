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

    # watch it fly, calm air, default (Module 2) camera
    ros2 launch quad_px4 sitl.launch.py

    # the recommended camera, no disturbance, recorded
    ros2 launch quad_px4 sitl.launch.py camera:=module3wide_2304 headless:=true rosbag:=true

## Arguments

### Session

| argument | default | what it does |
| --- | --- | --- |
| `headless` | `false` | `true` drops the Gazebo GUI. Sensor rendering still runs, and RTF is no longer pinned to 1.0. |
| `controllers` | `true` | `false` skips `control.launch.py` after the gates — the aircraft takes off and loiters. Use it to check takeoff, camera framing and lock without handover. |
| `rosbag` | `false` | `true` records the topic set in `quad_utils/config/bag_topics.yaml` to `bags/px4_<timestamp>`, mcap, with `--use-sim-time`. Recording starts at handover, not at launch. |
| `foxglove` | `false` | `true` starts the Foxglove bridge alongside the TF/marker visualisation. |
| `px4_dir` | `~/Robotics/fxteso_ibvs/external/PX4-Autopilot` | The PX4 fork holding `build/px4_sitl_default`. Launch fails loudly if the binary is missing. |
| `xrce_agent` | `~/Robotics/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent` | The uXRCE-DDS agent binary. It is not normally on `PATH`. |

### Camera and target geometry

`camera` selects a Raspberry Pi module **and a sensor mode** from
`quad_gz_sim/config/cameras.yaml` — one table that feeds both the `<camera>` block gz-sim
renders and `image_features`' feature model, so the two cannot disagree. Modes are separate
presets because not every module reaches the 50 Hz loop at full field of view.

| preset | hfov | render | fps | note |
| --- | --- | --- | --- | --- |
| `module2_1640` | 62.2° | 820×616 | 41.9 | **default.** The thesis camera; every archived bag was flown on it. Below the 50 Hz loop. |
| `module2_1640_native` | 62.2° | 1640×1232 | 41.9 | Same optics at native resolution — the one preset that breaks the half-resolution rule. |
| `module2_1640_8bit` | 62.2° | 820×616 | 83.7 | 8-bit readout; the only way IMX219 clears 50 Hz without losing FOV. |
| `module2_1920` | 38.9° | 960×540 | 47.6 | Cropped mode. Kept to make the FOV cost measurable, not because it is usable. |
| `module3_2304` | 66.0° | 1152×648 | 56.0 | Module 3. Wider horizontally, **narrower vertically** — and the short axis binds. A downgrade here. |
| `module3wide_2304` | 102.0° | 1152×648 | 56.0 | Module 3 Wide. Full FOV above 50 Hz. Barrel coefficients are an assumption, not a calibration. |
| `module3wide_2304_ideal` | 102.0° | 1152×648 | 56.0 | The same with distortion zeroed. Not a real camera — the control arm separating FOV from lens. |
| `module3wide_1536` | 78.9° | 768×432 | 120.1 | High-rate fallback, cropped. Its coefficients overstate edge distortion at this crop. |

A preset with non-zero distortion pulls in the `camera_distort` node automatically and
repoints `image_features` at `/quad/camera/image_distorted`; a zero vector leaves the path on
`/quad/camera/image_raw`. Nothing to pass either way.

| argument | default | what it does |
| --- | --- | --- |
| `camera` | `module2_1640` | Preset from the table above. An unknown name fails at launch with the valid list. |
| `target_scale` | `1.0` | Scales the ArUco target. Smaller markers buy FOV slack and cost decode pixels; `aD` follows automatically. |
| `marker_dict` | `7x7` | `7x7` (the thesis, and every archived bag) or `4x4`, which decodes at about two thirds the pixel size. Same IDs, so corner ordering is unchanged. |
| `camera_rate` | `0.0` | Above zero, overrides the sensor's update rate — use it to fly the real mode's fps against the 50 Hz loop instead of the sim's free 50. |
| `zD` | `2.5` | Servoing depth. `aD`, `MIS_TAKEOFF_ALT` and the takeoff gate are all derived from it, so it is one number, not three. |

### Disturbance

Magnitudes live in `quad_gz_sim/config/disturbances.yaml`; the scale arguments multiply them
so a sweep varies one number while the YAML keeps owning the shape. `/disturbances` is an
inertial-frame force in newtons on a 2 kg airframe, held until `pos_ctrl` publishes.

| argument | default | what it does |
| --- | --- | --- |
| `disturbance` | `none` | `none`, `step`, `gust`, `wind`, `csv`. Profiles **compose**: `disturbance:=wind,step`. |
| `disturbance_seed` | `0` | RNG seed for `gust` and for `wind`'s turbulence. Same seed, same realisation. |
| `gust_scale` | `1.0` | Multiplies `gust_sigma` (0.10, 0.10, 0.05 N). |
| `wind_scale` | `1.0` | Multiplies `wind_velocity` (0.8, 0.4, 0.0 m/s) **and its turbulence**. Drag is quadratic, so quote realised force, never the scale. |
| `gust_tau` | `1.5` | The gust's correlation time in seconds. Sweep it against the estimator lag. |

### Target trajectory

| argument | default | what it does |
| --- | --- | --- |
| `target_profile` | `thesis` | `thesis`, `hover`, `line`, `circle`, `steps`. `thesis` is the archived trajectory: it ends near t = 165 s with hover to ~260 s. |
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
| `takeoff_alt` | *(empty → `zD`)* | Depth at handover. Setting it away from `zD` makes the initial depth error a swept variable rather than an accident. One resolver feeds PX4, the bridge and the gate, so they cannot disagree. |
| `takeoff_tolerance` | `0.10` | How close to `takeoff_alt` the gate insists on. The node's own 0.5 m default straddles the depth stability threshold. |
| `estimators_ready` | `5.0` | Seconds of settled hover held before handover. EKF2's pitch reads ~0.019 rad high early; commanding zero against that is uncommanded forward acceleration into the tight FOV axis. |
| `max_feature_error` | `0.15` | Marker alignment required before handover. `\|qpsi\|` at handover separates held from collapsed runs at ~0.17; `0` disables the test. |

Both gates carry wall-clock timeouts that are **not** launch arguments — 180 s on takeoff,
120 s on lock. A gate that times out logs why and the stage after it never starts.

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

Not exposed here, but available on `estimation.launch.py` directly:
`initial_estimate_offset`, the seeded initial estimation error `(qx,qy,qz,qpsi)`.

## Recipes

    # calm air on the wide camera, recorded, no GUI
    ros2 launch quad_px4 sitl.launch.py camera:=module3wide_2304 headless:=true rosbag:=true

    # separate what the field of view buys from what the assumed barrel profile costs
    ros2 launch quad_px4 sitl.launch.py camera:=module3wide_2304_ideal

    # gust, seeded so the realisation repeats
    ros2 launch quad_px4 sitl.launch.py disturbance:=gust disturbance_seed:=7 gust_scale:=2.0

    # steady wind plus a pulse, composed
    ros2 launch quad_px4 sitl.launch.py disturbance:=wind,step wind_scale:=1.5

    # takeoff and camera framing only, no handover
    ros2 launch quad_px4 sitl.launch.py controllers:=false

    # a deliberate initial depth error at handover
    ros2 launch quad_px4 sitl.launch.py zD:=2.5 takeoff_alt:=2.0

    # hold the target still and sweep the observer instead
    ros2 launch quad_px4 sitl.launch.py target_profile:=hover observer_omega:=6.0

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
