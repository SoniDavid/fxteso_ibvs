"""The analytic or gazebo stack by plant:=, with sitl.launch.py's defaults. Thesis conditions:
sim_stack.py's THESIS set, passed explicitly.

  ros2 launch quad_utils sim.launch.py [plant:=analytic|gazebo] [headless:=true]
                                       [rosbag:=true] [foxglove:=true] [controllers:=false]
                                       [disturbance:=none|step|gust|wind|table52|csv]
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
    return _stack().build(bag_prefix='ibvs_')
