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

### Venue Indoor, Outdoor and Vicon argument

The `venue` argument toggles the simulation to mirror either a GPS-denied lab or an open outdoor environment:

* **`venue:=indoor` (Default)**: Simulates an indoor lab where GPS signals are unavailable.
  * **Sensors**: Relies entirely on the barometer for height; no GNSS aiding.
  * **Takeoff Flow**: `sim_pilot` arms -> Altitude Mode -> Climbs to `takeoff_alt` -> Offboard Mode (autonomous servoing).
  * *Note: SITL uses MAVLink RC for `sim_pilot`, but real hardware requires an RC transmitter (`COM_RC_IN_MODE=0`).*

* **`venue:=outdoor`**: Simulates an outdoor environment with clear skies for full GPS availability.
  * **Sensors**: Fuses GNSS for full 3D position and heading hold.
  * **Takeoff Flow**: Auto-arms -> Auto-takeoff (Position Mode) -> Climbs to `takeoff_alt` -> Offboard Mode (autonomous servoing).

* **`venue:=vicon`**: The indoor lab **with a Vicon volume aiding EKF2**. Everything `indoor` does,
  plus `EKF2_EV_CTRL=9` (horizontal position and yaw; height stays on the barometer), `vicon_sim`
  standing in for the mocap system, and `vicon_px4_bridge` — the same node that flies on hardware.
  * **Aiding is gated on OFFBOARD.** EKF2 is aided while the pilot has the aircraft and cut the
    moment `px4_offboard_bridge` asks for OFFBOARD, so the scored window is unaided and no ground
    truth is inside the loop under test. `aiding_policy:=always` removes the gate, which is the
    A/B that asks whether the indoor blocker is the estimator.
  * `vicon_latency`, `vicon_noise_sd` and `vicon_dropout_probability` shape what `vicon_sim`
    imitates. The defaults are deliberately non-zero: a bridge only ever tested on a clean feed
    proves nothing about the one that meets a lab.
  * `EKF2_EV_CTRL` is set explicitly on **all three** branches, `0` for `indoor` and `outdoor`.
    PX4 persists parameters into `parameters.bson`, so a stale `9` would silently aid an indoor
    run that is supposed to be unaided.

```sh
ros2 launch quad_px4 sitl.launch.py venue:=indoor
ros2 launch quad_px4 sitl.launch.py venue:=vicon                       # gated, the flight config
ros2 launch quad_px4 sitl.launch.py venue:=vicon aiding_policy:=always # the A/B arm
```

Whether the stream was actually **fused** is not visible from the ROS side — a published but
rejected stream looks identical. Check `cs_ev_pos` in `/fmu/out/estimator_status_flags`
(`cs_ev_yaw` stays false: yaw is deliberately not fused), or in the ulog.

Worse, a stream in the **wrong frame** looks identical from inside EKF2 too — without GNSS it
re-anchors to whatever it is handed, so a 90° rotation reads as zero innovation. That is what
`vicon_px4_bridge`'s frame check catches: it refuses to publish until the converted Vicon attitude
agrees with EKF2's own to within 30°, and names the likely wrong knob.

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

One launch file per plant, and one body for all three. `quad_description/launch/airframe.py` derives the F450's composite mass, centre of mass and inertia from `quad_mass`, and every plant integrates that body:

| plant | actuation | integrator | what it is for |
| --- | --- | --- | --- |
| `analytic` | ideal thrust + torque at the centre of mass | Euler, 100 Hz (`uav_dynamics`); Gazebo only renders | the thesis equations as written |
| `gazebo` | the same ideal wrench (BodyWrench), rotors welded | DART, 1 kHz | the same model in a real physics engine, in SITL's world and camera |
| `px4` | PX4 allocation → spinning rotors (MulticopterMotorModel) | DART, 1 kHz | the platform that transfers to hardware |

Both take `sitl.launch.py`'s arguments wherever the argument means the same thing — camera and detector, target trajectory, disturbance, observer and controller gains, gates, recording — with its deployment defaults: zD 1.2, the 0.5-scale target on `hover`, held until the loop takes over, and the 1.30 kg 3S build. Unlike SITL they record from launch (`record_from:=launch`), so a run that never locks still leaves a bag.

```sh
ros2 launch quad_utils analytic.launch.py
ros2 launch quad_utils gazebo.launch.py target_profile:=line target_heading:=90 disturbance:=wind

# the thesis conditions (the archived specs pass exactly this)
ros2 launch quad_utils sim.launch.py plant:=analytic zD:=2.5 target_scale:=1.0 target_profile:=thesis \
    quad_mass:=2.0 start_altitude:=4.0 hold_target:=false record_from:=handover sim_hold:=false
```

`sim.launch.py` is the same stack with `plant:=` as an argument, and every entry point here defaults to SITL's profile.

**Lock loss.** `pos_ctrl` stops on lock loss and leaves the aircraft to PX4. On these plants `sim_hold` stands in for it, with SITL's timing: after 0.3 s without `pos_ctrl` plus `COM_OF_LOSS_T` (1.0 s) of the last command, it holds position with PX4's position cascade and the fork's default gains, and it hands back the moment `pos_ctrl` publishes again. `/sim_hold/active` is bagged, so these runs score on authority gaps exactly as SITL does. `sim_hold:=false` restores the old behaviour, for A/B only.

**Gazebo's native wind is not used.** gz-sim 8's WindEffects applies a linear drag and noise to the airframe even at zero wind, so it is kept out of `gazebo`. Every disturbance comes through `disturbance:=`, as in `analytic`. It is still present under `px4`.

Plant-specific arguments: `start_altitude` (default 1.5) replaces the takeoff — the aircraft starts over the target at that height and descends onto zD; `unwrap_attitude` is gazebo only. PX4-only, and not declared here: `venue`, `vicon_*`, `aiding_policy`, `ev_velocity`, `imu_ctrl`, `mag_acclim`, `prop`, `battery_cells`, `hover_thrust`, `takeoff_*`, `offboard_recovery`, `pilot_*`, `attitude_oracle`. `velocity_source:=ekf2` on these plants reads the plant's own velocity, which is truth.

`att_ctrl`'s inertia is the thesis vehicle's, so at any `quad_mass` other than 2.0 it flies a body it does not model exactly (0.69× the inertia at 1.30 kg). That mismatch belongs to the control law, which is left as it is.

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
