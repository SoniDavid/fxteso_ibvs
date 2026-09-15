# FXTESO_IBVS

Fixed-Time Extended State Observer — SMC Image Based Visual Servoing for quadrotor.

ROS2 Jazzy · Gazebo Harmonic · PX4 Autopilot fork · Raspberry Pi 5 companion computer

## Packages

- **`quad_common`**: Shared headers (`SimRate`, `Unwrapped`).
- **`quad_control`**: FxTESO, differentiators, ArUco extractor (`aruco_nano` / OpenCV), SMC loops. Ships to hardware.
- **`quad_description`**: Geometry, camera, and flight sensor assets, and the camera preset table sim and hardware share.
- **`quad_gz_sim`**: Simulation world, ROS-GZ bridge, plant backends, and scenario generation. Desktop only.
- **`quad_px4`**: PX4 adapter, OFFBOARD bridge, takeoff gate, and the SITL and hardware launch files.
- **`quad_cam`**: Raspberry Pi Camera Module 3 Wide driver, bench launch and probes. Ships to hardware.
- **`quad_utils`**: Launch files, transforms, and visualization configurations.

Dependencies: `{quad_utils, quad_px4} → {quad_control, quad_gz_sim, quad_cam} → {quad_description, quad_common}`

## Submodules

Pinned for SITL compatibility — the DDS topics and SITL gains are only valid against a specific PX4:

- **`src/px4_msgs`** (`PX4/px4_msgs`): uXRCE-DDS topic versions must match the flown PX4.
- **`external/PX4-Autopilot`** (`SoniDavid/PX4-Autopilot`): Fork carrying the `22100_gz_F450_px4` airframe (`v1.18.0-beta1-271`).

```sh
git submodule update --init src/px4_msgs                          # always
git submodule update --init --recursive external/PX4-Autopilot    # only for PX4 SITL
```

## Build & Run

Both machines need OpenCV ≥ 4.7 and `cv_bridge` rebuilt against it first — see [**RUNNING.md**](RUNNING.md).

```sh
# dev desktop
colcon build --symlink-install --allow-overriding cv_bridge image_geometry && source install/setup.bash

# PX4 SITL (one-off build, ~1.6 GB)
make -C external/PX4-Autopilot px4_sitl_default

# Raspberry Pi 5 - no simulator, no sim_pilot
colcon build --symlink-install --packages-skip quad_gz_sim \
    --allow-overriding cv_bridge image_geometry --cmake-args -DBUILD_SIM_PILOT=OFF
```

For all launch commands, arguments, and deployment configurations see [**RUNNING.md**](RUNNING.md).

## PX4 SITL — Architecture Notes

The control stack (`image_features` → `fixed_eso` → `pos_ctrl`) is the same code all plants run, and the same code the UAV flies. `att_ctrl` does not start — PX4's `mc_att_control`/`mc_rate_control` own the inner loop. `pos_ctrl` emits a desired attitude and collective thrust, forwarded by `px4_offboard_bridge` as an OFFBOARD `VehicleAttitudeSetpoint`.

PX4 attaches in standalone mode (`PX4_GZ_STANDALONE`, `PX4_GZ_MODEL_NAME`) so the world, target, and disturbance injection are shared with the other plants.

Notable PX4-specific details (each commented where it lives):

- **`MPC_THR_HOVER`**: Measured per battery/prop combination on a real hover — the vehicle now flies 3S, not the 4S `sitl.launch.py`'s default (`battery_cells`, `hover_thrust`) assumes. The affine rotor-velocity map means the nominal x500 value (0.60) would silently mis-scale every command regardless of cell count.
- **`frame_yaw_offset`**: Appears twice, opposite signs — the world is ENU, the workspace calls Gazebo +x North.
- **Estimator startup**: `px4_takeoff_gate` gates the estimators; starting earlier books the climb as disturbance.
- **Sensor plugins**: `worlds/ibvs.sdf` carries PX4's plugins itself — gz-sim ignores `server.config` once any `<plugin>` is declared.
