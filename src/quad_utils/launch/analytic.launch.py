"""FxTESO + adaptive-gain SMC IBVS on the analytic plant: uav_dynamics integrates the aircraft,
Gazebo only renders the camera. SITL's arguments and defaults; see sim_stack.py.

  ros2 launch quad_utils analytic.launch.py [headless:=true] [rosbag:=true] [disturbance:=...]

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
