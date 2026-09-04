# FXTESO_IBVS

Fixed-Time Extended State Observer - SMC Image Based Visual Servoing for quadrotor

ROS2 Jazzy - Gazebo Harmonic

## Packages

| Package | What it holds |
| --- | --- |
| `quad_common` | Header-only shared code: `SimRate` (sim-time loop pacing) and `Unwrapped` (unbounded Euler angles). |
| `quad_control` | The FxTESO, the tracking differentiators, the ArUco feature extractor and both adaptive-gain SMC loops. No simulation dependency of any kind - this is the package that ships to the aircraft. |
| `quad_description` | ArUco target plus the F450: `F450_base` is the shared geometry, camera and flight sensors; `F450` adds cosmetic rotor spin, `F450_px4` adds the motor model PX4 drives. Assets only. |
| `quad_gz_sim` | Simulation only: the world, the ros_gz bridge, the analytic and DART plant backends, the scenario generators and the `BodyWrench` gz-sim plugin. |
| `quad_px4` | PX4 SITL backend: state adapter, OFFBOARD setpoint bridge and takeoff gate. The F450 airframe lives in the PX4 fork, since PX4 only reads airframes from its own ROMFS. |
| `quad_utils` | Bringup and visualisation: the composed launch files, `tf_broadcaster`, the Foxglove layout and the rosbag topic list. |

Dependencies run one way: {`quad_utils`, `quad_px4`} → {`quad_control`, `quad_gz_sim`} → {`quad_description`, `quad_common`}.

## Submodules

Two dependencies are pinned rather than tracked upstream, because the SITL gains are only
valid against a specific PX4:

| Path | Repo | Why pinned |
| --- | --- | --- |
| `src/px4_msgs` | `PX4/px4_msgs` | The uXRCE-DDS topic versions must match the PX4 that is flown. |
| `external/PX4-Autopilot` | `SoniDavid/PX4-Autopilot` | Fork carrying the `22100_gz_F450_px4` airframe. Forked from upstream `ea63910683` (`v1.18.0-beta1-271`). |

`src/px4_msgs` is small. `external/PX4-Autopilot` is ~1.6 GB with 35 nested submodules and
is only needed for `plant:=px4`, so it is worth cloning shallowly:

```sh
git submodule update --init src/px4_msgs                          # always
git submodule update --init --recursive external/PX4-Autopilot    # only for PX4 SITL
```

`external/` carries a `COLCON_IGNORE`: PX4 ships a root `package.xml`, so colcon would
otherwise try to build it.

## Running

```sh
colcon build --symlink-install && source install/setup.bash

# the full closed loop
ros2 launch quad_utils sim.launch.py [headless:=true] [rosbag:=true] [foxglove:=true] \
                                     [plant:=analytic|gazebo] [controllers:=false] \
                                     [disturbance:=none|step|gust|wind|table52|csv] [disturbance_seed:=N] \
                                     [gust_scale:=1.0] [wind_scale:=1.0] [turbulence_scale:=1.0] \
                                     [camera:=module2_1640] [camera_rate:=0.0] \
                                     [target_scale:=1.0] [marker_dict:=7x7|4x4] [zD:=2.5]

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
# one-off: fetch the pinned PX4 fork (~1.6 GB, 35 nested submodules) and build it
git submodule update --init --recursive external/PX4-Autopilot
make -C external/PX4-Autopilot px4_sitl_default

ros2 launch quad_px4 sitl.launch.py [headless:=true] [rosbag:=true] [foxglove:=true]
                                    [controllers:=false] [record_from:=handover|launch]
                                    [disturbance:=none|step|gust|wind|table52|csv]
                                    [turbulence_scale:=1.0]
                                    [camera:=module3wide_2304] [target_scale:=0.5] [zD:=1.2]
                                    [px4_dir:=...] [xrce_agent:=...]
```

`RUNS.md` at the repository root lists the standing configurations. `cleanup.sh` kills a
previous run's processes, which has to happen before starting another one.

The high-level stack is unchanged: `image_features` → `fixed_eso` → `pos_ctrl` is the same
code the other plants run, with the same gains. `att_ctrl` does not start - PX4's
`mc_att_control` and `mc_rate_control` take the inner loop. `pos_ctrl` already emits a
desired attitude and a collective thrust, which `px4_offboard_bridge` forwards as an
OFFBOARD `VehicleAttitudeSetpoint`.

Gazebo is ours; PX4 attaches to it in standalone mode (`PX4_GZ_STANDALONE`,
`PX4_GZ_MODEL_NAME`) rather than starting its own server, so the world, the target and the
disturbance injection are the same ones the other plants use.

Five things are specific to this plant, each commented where it lives:

- `MPC_THR_HOVER` is the *measured* hover throttle (0.716), not x500's nominal 0.60. PX4's
  `CT*u^2` thrust model is exact only at full throttle, because `SIM_GZ_EC_MIN` idles the
  rotors at 150 rad/s and the map to rotor velocity is affine; the hover anchor absorbs it.
- `CA_ROTOR*_KM` and `_CT` now match the SDF rather than x500's inherited 0.05, but both are
  inert: `ActuatorEffectivenessRotors` sets `normalize_rpy`, so each column of the mix is
  divided by its own maximum and any common scale on it cancels.
- `frame_yaw_offset` appears twice, same value, opposite directions: the world is ENU while
  this workspace calls Gazebo +x North.
- The estimators wait for `px4_takeoff_gate`; started earlier they book the climb as
  disturbance.
- `worlds/ibvs.sdf` carries PX4's sensor plugins itself - gz-sim ignores PX4's
  `server.config` once a world declares any `<plugin>` of its own.
- The attitude and rate gains are inertia-and-arm scaled off x500's; roll is the soft axis.

### The camera

A `camera:=` preset is a Raspberry Pi module **and a sensor mode**, from
`quad_gz_sim/config/cameras.yaml`, because not every module reaches the 50 Hz `image_features`
loop at full field of view. One preset rewrites the `<camera>` block of a derived `F450_base`
**and** feeds `image_features` the same intrinsics, so the rendered camera and the feature model
cannot drift apart. `zD:=` likewise drives `aD` and PX4's `MIS_TAKEOFF_ALT` together:

    aD = 0.5625 * target_scale^2 * (0.00304 / zD)^2

It is `aD`, not `zD`, that decides where the aircraft settles.

### Defaults

`sitl.launch.py` defaults to `module3wide_2304`, takeoff at 1.5 m, `zD` 1.2, `target_scale` 0.5
and `target_profile` hover, so a bare launch takes off, descends onto a stationary target and
holds. Every one of them is overridable; `RUNS.md` lists the configurations that get used.

`sim.launch.py` and `observer_only.launch.py` keep `zD` 2.5 and `target_scale` 1.0 — `takeoff_alt`
is a PX4 concept and neither of those plants takes off. The camera preset is shared by all three.

### Disturbances

`disturbance:=` selects one or more profiles, and they compose (`disturbance:=wind,step`):

| profile | what it injects |
| --- | --- |
| `none` | nothing |
| `step` | a rectangular force pulse over a fixed interval |
| `gust` | an Ornstein-Uhlenbeck force with its own correlation time (`gust_tau`) |
| `wind` | a wind velocity through the quadratic drag law, optionally with turbulence on it |
| `table52` | a scheduled mean wind with Von Karman turbulence on top (below) |
| `csv` | a recorded profile replayed, one `x,y,z` row per 10 ms |

Magnitudes live in `quad_gz_sim/config/disturbances.yaml`; `gust_scale` and `wind_scale`
multiply them so a sweep is one number on the launch line. The force is held at zero until
after controller handover, so it lands on a servoing aircraft rather than on a takeoff.

`table52` is a four-interval mean schedule whose direction reverses and which includes a
downdraft, switched off after `table52_end` so the state can be seen converging afterwards. Von
Karman turbulence is layered on it through shaping filters whose coefficients follow airspeed
and altitude. The schedule and the turbulence intensities are in `disturbances.cpp` rather than
in the YAML, because they define the profile; the on/off times, the altitude and
`turbulence_scale` are parameters. `turbulence_scale:=0.0` flies the mean schedule alone.

