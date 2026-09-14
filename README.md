# FXTESO_IBVS

Fixed-Time Extended State Observer — SMC Image Based Visual Servoing for quadrotor.

ROS2 Jazzy · Gazebo Harmonic · PX4 Autopilot fork

## Packages

- **`quad_common`**: Shared headers (`SimRate`, `Unwrapped`).
- **`quad_control`**: FxTESO, differentiators, ArUco extractor, SMC loops. Ships to hardware.
- **`quad_description`**: Geometry, camera, and flight sensor assets.
- **`quad_gz_sim`**: Simulation world, ROS-GZ bridge, plant backends, and scenario generation.
- **`quad_px4`**: PX4 SITL adapter, OFFBOARD bridge, and takeoff gate.
- **`quad_utils`**: Launch files, transforms, and visualization configurations.

Dependencies: `{quad_utils, quad_px4} → {quad_control, quad_gz_sim} → {quad_description, quad_common}`

## Submodules

Pinned for SITL compatibility — the DDS topics and SITL gains are only valid against a specific PX4:

- **`src/px4_msgs`** (`PX4/px4_msgs`): uXRCE-DDS topic versions must match the flown PX4.
- **`external/PX4-Autopilot`** (`SoniDavid/PX4-Autopilot`): Fork carrying the `22100_gz_F450_px4` airframe (`v1.18.0-beta1-271`).

```sh
git submodule update --init src/px4_msgs                          # always
git submodule update --init --recursive external/PX4-Autopilot    # only for PX4 SITL
```

## Build & Run

```sh
colcon build --symlink-install && source install/setup.bash

# PX4 SITL (one-off build, ~1.6 GB)
make -C external/PX4-Autopilot px4_sitl_default
```

For all launch commands, arguments, and deployment configurations see [**RUNNING.md**](RUNNING.md).

## PX4 SITL — Architecture Notes

The control stack (`image_features` → `fixed_eso` → `pos_ctrl`) is the same code all plants run. `att_ctrl` does not start — PX4's `mc_att_control`/`mc_rate_control` own the inner loop. `pos_ctrl` emits a desired attitude and collective thrust, forwarded by `px4_offboard_bridge` as an OFFBOARD `VehicleAttitudeSetpoint`.

PX4 attaches in standalone mode (`PX4_GZ_STANDALONE`, `PX4_GZ_MODEL_NAME`) so the world, target, and disturbance injection are shared with the other plants.

Notable PX4-specific details (each commented where it lives):

- **`MPC_THR_HOVER`**: Measured at 0.6461 for 4S on 9545 props. The affine rotor-velocity map means the nominal x500 value (0.60) would silently mis-scale every command.
- **`frame_yaw_offset`**: Appears twice, opposite signs — the world is ENU, the workspace calls Gazebo +x North.
- **Estimator startup**: `px4_takeoff_gate` gates the estimators; starting earlier books the climb as disturbance.
- **Sensor plugins**: `worlds/ibvs.sdf` carries PX4's plugins itself — gz-sim ignores `server.config` once any `<plugin>` is declared.
