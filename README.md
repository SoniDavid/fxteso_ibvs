# FXTESO_IBVS

Fixed-Time Extended State Observer - Image Based Visual Servoing for quadrotor

Currently on ROS2 Jazzy - Gazebo Harmonic

## Packages

| Package | What it holds |
| --- | --- |
| `quad_common` | Header-only shared code. Currently just `SimRate`, the sim-time loop pacing. |
| `quad_control` | The FxTESO, the tracking differentiators, the ArUco feature extractor and both adaptive-gain SMC loops. No simulation dependency of any kind - this is the package that ships to the aircraft. |
| `quad_description` | F450 and ArUco target SDF models and meshes. Assets only. |
| `quad_gz_sim` | Simulation only: the world, the ros_gz bridge, both plant backends, the scenario generators and the `BodyWrench` gz-sim plugin. |
| `quad_utils` | Bringup and visualisation: the composed launch files, `tf_broadcaster`, the Foxglove layout and the rosbag topic list. |

Dependencies run one way: `quad_utils` → {`quad_control`, `quad_gz_sim`} → {`quad_description`, `quad_common`}.

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

The plant is switchable and both backends publish the same five state topics, so nothing
downstream knows which one ran:

- `plant:=analytic` - `uav_dynamics.cpp` integrates the aircraft; Gazebo is a camera only.
- `plant:=gazebo` - DART integrates it; `gz_state_adapter` converts the odometry to NED.

## TO DO:
- Write setup.sh
- Include PX4 SITL
    - Use standard PX4 quadrotor controllers only with high-level observer
    - Rewrite low level controllers to use ASMC
    - Drops in as a `quad_px4` package replacing `quad_gz_sim/launch/plant.launch.py`:
      any adapter publishing the same five state topics needs no change downstream.
- Test High-level observer on real stack (after SITL)
- Test low and high level controllers simulatneously
