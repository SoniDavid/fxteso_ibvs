# Running in simulation

The dev-desktop / PX4 SITL side of [RUNNING.md](RUNNING.md); shared setup, the launch argument reference, and camera presets live there. For the real UAV and the Raspberry Pi, see [RUNNING_HW.md](RUNNING_HW.md).

## Dev desktop (SITL)

```sh
# Cloning the repository
git clone https://github.com/SoniDavid/fxteso_ibvs
git submodule update --init --recursive

# OpenCV + cv_bridge first (RUNNING.md), then building
colcon build --symlink-install --allow-overriding cv_bridge image_geometry

# Sourcing
source /opt/ros/jazzy/setup.bash
source ~/[path_to_project]/fxteso_ibvs/install/setup.bash
```

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

# the detector A/B: OpenCV's ArUco alone instead of the hybrid, everything else equal
ros2 launch quad_px4 sitl.launch.py detector_backend:=opencv
```

`image_features` logs `detectMarkers (<backend>) <mean> ms mean / <worst> ms worst` every 10 s, and every 50 Hz loop logs a `loop: target ... | achieved ...` line — at WARN when it overran since the last one.

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

`controllers:=false` so nothing acts on the estimate and what is measured is a property of the observer alone. `record_from:=launch` because `fixed_eso` starts at the takeoff gate and converges in about a second — a recorder started at handover misses the transient entirely.

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

Visualisation against a recorded bag. Pass the camera preset: the layout to import depends on which image topic `image_features` consumes, and the launch logs the path of the right one.

```sh
ros2 bag play bags/px4_20260826_120000 --clock
ros2 launch quad_utils viz.launch.py foxglove:=true camera:=module3wide_2304
```

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

Magnitudes live in `quad_gz_sim/config/disturbances.yaml`; `gust_scale` and `wind_scale` multiply them so a sweep is one number on the launch line. The force is held at zero until after controller handover, so it lands on a servoing aircraft, not on a takeoff.

`table52` is a four-interval mean schedule with direction reversals and a downdraft. Von Karman turbulence is layered on top through shaping filters. `turbulence_scale:=0.0` flies the mean schedule alone.
