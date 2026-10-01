"""FxTESO + adaptive-gain SMC IBVS on the analytic plant: uav_dynamics.cpp integrates the
aircraft and Gazebo only renders the camera.

  quad_gz_sim/scenario  -> tgt_* and /disturbances
  quad_gz_sim/plant     -> uav_dynamics -> quad_position/quad_attitude/...
        |                        gz_pose_broadcaster --> Gazebo (renderer only)
        v                                                    |
  quad_control/estimation <------------- /quad/camera/image_raw
        |
        +---> fixed_eso ---> pos_ctrl ---> att_ctrl ---> quad_torques/quad_thrust

Takes sitl.launch.py's arguments wherever they mean the same thing, with its defaults: zD 1.2,
the 0.5-scale target holding station, the 1.30 kg 3S build. Arguments: sim_stack.py.

  ros2 launch quad_utils analytic.launch.py [headless:=true] [rosbag:=true] [foxglove:=true]
                                            [camera:=...] [target_profile:=hover|line|...]
                                            [disturbance:=none|step|gust|wind|table52|csv]
                                            [quad_mass:=1.30] [start_altitude:=1.5]

Thesis conditions: zD:=2.5 target_scale:=1.0 target_profile:=thesis quad_mass:=2.0
start_altitude:=4.0 hold_target:=false record_from:=handover sim_hold:=false.
"""
import importlib.util
import os


def _stack():
    """sim_stack.py beside this file; launch/ is not on sys.path, so it is loaded by path."""
    path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sim_stack.py')
    spec = importlib.util.spec_from_file_location('sim_stack', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod



def generate_launch_description():
    return _stack().build(plant='analytic', bag_prefix='analytic_')
