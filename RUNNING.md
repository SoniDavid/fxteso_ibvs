# Running the FxTESO-IBVS project

## Setup the environment

The project currently uses dependancies such as:
- ROS2 Jazzy Jalisco
- Gazebo Sim 8.11 - Harmonic
- PX4 Autopilot fork

Be sure to have them installed before trying to build and run the project


```sh
# Cloning the repository 
git clone https://github.com/SoniDavid/fxteso_ibvs
git submodule update --init --recursive

# Building
colcon build --symlink-install

# Sourcing
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash
source ~/[path_to_project]}/fxteso_ibvs/install/setup.bash
```

## Launch Arguments Reference

You can configure the runs by appending these arguments to the `ros2 launch` commands. Below is an exhaustive list of all arguments available in `sitl.launch.py` (and a few from `hardware.launch.py` or `sim.launch.py`).

### Display & Recording
- `headless:=<bool>` — Drop the Gazebo window (default: `false`).
- `rosbag:=<bool>` — Record a bag of the run (default: `false`).
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
- `target_scale:=<float>` — Visual scale of the target (default: `0.5`).
- `marker_dict:=<str>` — ArUco marker dictionary (default: `7x7`).
- `blackout_at:=<float>` — Time in seconds to hide the target (default: `0.0`).
- `blackout_for:=<float>` — Duration in seconds to hide the target (default: `3.0`).

### Flight Geometry & Observer
- `zD:=<float>` — Servoing depth (default: `1.2`).
- `takeoff_alt:=<float|zD>` — Takeoff altitude. Pass literal `zD` to launch directly at the servoing depth (default: `1.5`).
- `takeoff_tolerance:=<float>` — Required proximity to `takeoff_alt` before gate opens (default: `0.10`).
- `eso_z_des:=<float>` — Observer's desired servoing depth (defaults to tracking `zD`).
- `observer_omega:=<float>` — If > 0, sets fixed_eso's x/y gains as a triple pole at this rate (default: `0.0`).
- `gamma1_xy`...`gamma4_yaw` — Various observer gains. Defaults: `gamma1_xy=18.0`, `gamma2_xy=20.0`, `gamma3_xy=4.0`, `gamma1_yaw=5.0`, `gamma2_yaw=16.0`, `gamma3_yaw=3.0`, `gamma4_yaw=0.001`, `alpha_yaw=0.75`, `beta_yaw=1.2`.
- `eso_yaw_sign:=<float>` — Yaw sign for the observer (default: `1.0`).
- `initial_estimate_offset:="[<qx>, <qy>, <qz>, <qpsi>]"` — Initial estimate error offset (default: `[0.0, 0.0, 0.0, 0.0]`).
- `estimators_ready:=<float>` — Settled hover time required before handover (default: `5.0`).
- `max_feature_error:=<float>` — Required marker alignment before pos_ctrl takes over (default: `0.15`).
- `gate_timeout:=<float>` — Timeout for the gate (default: `120.0`).
- `controllers:=<bool>` — Enable or disable controllers (default: `true`).

### Hardware & Plant Configurations
- `plant:=<analytic|gazebo>` — Plant model used in simulation (for `sim.launch.py`).
- `camera:=<preset>` — Camera config preset (e.g., `module2_1640`, `module3wide_2304`).
- `camera_rate:=<float>` — Camera rate (default: `0.0`).
- `prop:=<str>` — Rotor model (default: `9545`).
- `battery_cells:=<int>` — Battery cell count (default: `4`).
- `hover_thrust:=<float>` — Measured `MPC_THR_HOVER` (default: `0.6461`).
- `offboard_recovery:=<bool>` — Attempt offboard recovery on lock-loss (default: `false`).
- `px4_dir:=<path>` — Path to the PX4 fork submodule.
- `xrce_agent:=<path>` — Path to the MicroXRCEAgent binary.

## Running the Simulation 
Although the repository considers using _Gazebo's DART_ physics and _euler integrated analytical physics_, the **default sim environment** is _PX4 Software in the Loop (SITL)_ due to its higher fidelity to real world conditions. Furthermore, default deployment geometry considers **1.5 m takeoff, 1.2 m servoing**

Conditions where chose based on RSCL-ITESM (Robotics System Control Laboratory) real 450 UAV deployment constraints

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

### Venue Indoor and Outdoor argument

The `venue` argument toggles the simulation to mirror either a GPS-denied lab or an open outdoor environment:

* **`venue:=indoor` (Default)**: Simulates an indoor lab where GPS signals are unavailable.
  * **Sensors**: Relies entirely on the barometer for height; no GNSS aiding. 
  * **Takeoff Flow**: `sim_pilot` arms -> Altitude Mode -> Climbs to `takeoff_alt` -> Offboard Mode (autonomous servoing).
  * *Note: SITL uses MAVLink RC for `sim_pilot`, but real hardware requires an RC transmitter (`COM_RC_IN_MODE=0`).*

* **`venue:=outdoor`**: Simulates an outdoor environment with clear skies for full GPS availability.
  * **Sensors**: Fuses GNSS for full 3D position and heading hold.
  * **Takeoff Flow**: Auto-arms -> Auto-takeoff (Position Mode) -> Climbs to `takeoff_alt` -> Offboard Mode (autonomous servoing).

```sh
ros2 launch quad_px4 sitl.launch.py venue:=indoor
```

## Running only the Extended State Observer

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

## Camera Presets

A `camera:=` preset is a Raspberry Pi module **and a sensor mode**, from
`quad_gz_sim/config/cameras.yaml`. Not every module reaches the 50 Hz `image_features` loop at
full field of view. One preset rewrites the `<camera>` block in the SDF **and** feeds
`image_features` the same intrinsics — the rendered camera and the feature model cannot drift
apart. `zD:=` likewise drives `aD` and PX4's `MIS_TAKEOFF_ALT` together:

```
aD = 0.5625 * target_scale^2 * (0.00304 / zD)^2
```

It is `aD`, not `zD`, that decides where the aircraft settles.

**`sitl.launch.py` defaults**: `module3wide_2304`, takeoff 1.5 m, `zD` 1.2, `target_scale` 0.5,
`target_profile` hover — a bare launch takes off, descends onto a stationary target and holds.

**`sim.launch.py` / `observer_only.launch.py` defaults**: `zD` 2.5, `target_scale` 1.0 — neither
plant takes off so `takeoff_alt` is unused.

## Disturbance Profiles

`disturbance:=` selects one or more profiles; they compose (`disturbance:=wind,step`):

| Profile | What it injects |
| --- | --- |
| `none` | nothing |
| `step` | rectangular force pulse over a fixed interval |
| `gust` | Ornstein-Uhlenbeck force with correlation time `gust_tau` |
| `wind` | wind velocity through the quadratic drag law, optionally with turbulence |
| `table52` | scheduled mean wind with Von Karman turbulence on top |
| `csv` | recorded profile replayed, one `x,y,z` row per 10 ms |

Magnitudes live in `quad_gz_sim/config/disturbances.yaml`; `gust_scale` and `wind_scale`
multiply them so a sweep is one number on the launch line. The force is held at zero until after
controller handover, so it lands on a servoing aircraft, not on a takeoff.

`table52` is a four-interval mean schedule with direction reversals and a downdraft. Von Karman
turbulence is layered on top through shaping filters. `turbulence_scale:=0.0` flies the mean
schedule alone.
