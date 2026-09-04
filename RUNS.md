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
