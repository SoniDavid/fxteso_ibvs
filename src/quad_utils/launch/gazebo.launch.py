"""FxTESO + adaptive-gain SMC IBVS on the gazebo plant: DART integrates SITL's F450 (rotors
welded) under BodyWrench. SITL's arguments and defaults; see sim_stack.py.

  ros2 launch quad_utils gazebo.launch.py [headless:=true] [rosbag:=true] [disturbance:=...]
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
    return _stack().build(plant='gazebo', bag_prefix='gazebo_')
