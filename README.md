# FXTESO_IBVS

Fixed-Time Extended State Observer - Image Based Visual Servoing for quadrotor

Currently on ROS2 Jazzy - Gazebo Harmonic

## Packages

| Package | What it holds |
| --- | --- |
| `quad_common` | Header-only shared code: `SimRate` (sim-time loop pacing) and `Unwrapped` (unbounded Euler angles). |
| `quad_control` | The FxTESO, the tracking differentiators, the ArUco feature extractor and both adaptive-gain SMC loops. No simulation dependency of any kind - this is the package that ships to the aircraft. |
| `quad_description` | ArUco target plus the F450: `F450_base` is the shared geometry, camera and flight sensors; `F450` adds cosmetic rotor spin, `F450_px4` adds the motor model PX4 drives. Assets only. |
| `quad_gz_sim` | Simulation only: the world, the ros_gz bridge, the analytic and DART plant backends, the scenario generators and the `BodyWrench` gz-sim plugin. |
| `quad_px4` | PX4 SITL backend: state adapter, OFFBOARD setpoint bridge, takeoff gate and the F450 airframe definition. |
| `quad_utils` | Bringup and visualisation: the composed launch files, `tf_broadcaster`, the Foxglove layout and the rosbag topic list. |

Dependencies run one way: {`quad_utils`, `quad_px4`} → {`quad_control`, `quad_gz_sim`} → {`quad_description`, `quad_common`}.

## Running

```sh
colcon build --symlink-install && source install/setup.bash

# the full closed loop
ros2 launch quad_utils sim.launch.py [headless:=true] [rosbag:=true] [foxglove:=true] \
                                     [plant:=analytic|gazebo] [controllers:=false] \
                                     [disturbance:=none|step|gust|wind|csv] [disturbance_seed:=N]

# open loop: estimation chain only, controllers never start
ros2 launch quad_utils observer_only.launch.py [plant:=gazebo] [disturbance:=gust]

# visualisation alone, e.g. against a bag replay
ros2 launch quad_utils viz.launch.py foxglove:=true
```

Each layer also runs on its own, which is the point of the split:

```sh
ros2 launch quad_gz_sim gz_sim.launch.py    # world + bridge + pose broadcaster
ros2 launch quad_gz_sim plant.launch.py     # analytic integrator or DART adapter
ros2 launch quad_gz_sim scenario.launch.py  # moving target + disturbance force
ros2 launch quad_control estimation.launch.py
ros2 launch quad_control control.launch.py
```

The plant is switchable and every backend publishes the same five state topics, so nothing
downstream knows which one ran:

- `plant:=analytic` - `uav_dynamics.cpp` integrates the aircraft; Gazebo is a camera only.
- `plant:=gazebo` - DART integrates it; `gz_state_adapter` converts the odometry to NED.
- `plant:=px4` - DART integrates it and **PX4 SITL owns allocation, attitude and rates**;
  `px4_state_adapter` converts EKF2's estimate to the same five topics. Launched from
  `quad_px4`, not from `plant.launch.py`, because PX4 needs its own process and DDS agent.

## PX4 SITL

```sh
# one-off: symlink the airframe into the PX4 tree, then rebuild PX4
ros2 run quad_px4 link_px4_airframe.sh [/path/to/PX4-Autopilot]
make -C ~/Robotics/PX4-Autopilot px4_sitl_default

ros2 launch quad_px4 sitl.launch.py [headless:=true] [rosbag:=true] [foxglove:=true]
                                    [controllers:=false] [disturbance:=gust]
                                    [px4_dir:=...] [xrce_agent:=...]
```

The high-level stack is unchanged: `image_features` → `fixed_eso` → `pos_ctrl` is the same
code the other plants run, with the same gains. `att_ctrl` does not start - PX4's
`mc_att_control` and `mc_rate_control` take the inner loop. `pos_ctrl` already emits a
desired attitude and a collective thrust, which `px4_offboard_bridge` forwards as an
OFFBOARD `VehicleAttitudeSetpoint`.

Gazebo is ours; PX4 attaches to it in standalone mode (`PX4_GZ_STANDALONE`,
`PX4_GZ_MODEL_NAME`) rather than starting its own server, so the world, the target and the
disturbance injection are the same ones the other plants use.

Five things are specific to this plant, each commented where it lives:

- `MPC_THR_HOVER` is the *measured* hover throttle (0.716), not x500's nominal 0.60.
- `frame_yaw_offset` appears twice, same value, opposite directions: the world is ENU while
  this workspace calls Gazebo +x North.
- The estimators wait for `px4_takeoff_gate`; started earlier they book the climb as
  disturbance.
- `worlds/ibvs.sdf` carries PX4's sensor plugins itself - gz-sim ignores PX4's
  `server.config` once a world declares any `<plugin>` of its own.
- The attitude and rate gains are inertia-and-arm scaled off x500's; roll is the soft axis.

### Deviation from the thesis: `gamma3(3)`

`fixed_eso.cpp` uses **3** on the yaw channel of `gamma3`, where thesis Table 5.3 has 7.
Every other observer and controller gain is unchanged. The yaw channel is a triple integrator
with no absolute reference, and PX4's noisier estimate made it random-walk away.


A run that holds tracks the full 245 s to ~0.1 m. The remaining two failures are a different
mechanism - pos_ctrl's yaw command going marginally stable at the entry to the circular
phase - and are not yet fixed. The other plants are unaffected, re-measured at 100%
(analytic) and 99.9% (gazebo).

Two PX4 gain changes were tried and backed out: raising `MC_ROLL_P`/`MC_PITCH_P` cut the
attitude lag but tripled the image-moment noise, and raising `MC_YAW_P` made a stiffer loop
follow an oscillating command more eagerly. Both are commented at the gains themselves.

## Analysis tools

`quad_utils/analysis/` reads a `rosbag:=true` bag and needs no ROS graph, so it works on any
recorded run. `summary.py` is the one to reach for first; the others explain its columns.

```sh
cd src/quad_utils/analysis
python3 summary.py           ../../../bags/px4_* ../../../bags/ibvs_*
python3 attitude_tracking.py <bag>   # inner-loop error and lag - the PX4 tuning objective
python3 derotation_error.py  <bag>   # estimator error the virtual camera actually sees
python3 image_moments.py     <bag>   # qpsi moment noise and conditioning
python3 yaw_channel.py       <bag>   # noise and bias down the whole yaw chain
```

## TO DO:
- Write setup.sh
- PX4 SITL: 2 runs in 6 lose lock at the circular-phase entry; the ANFTIBVS yaw gains are
  the lever. The outcome is stochastic, so any attempt needs >=3 runs per configuration.
- Rewrite low level controllers to use ASMC
- Test High-level observer on real stack (after SITL)
- Test low and high level controllers simultaneously
