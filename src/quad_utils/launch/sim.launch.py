"""The analytic or gazebo stack by plant:=, at the thesis conditions: zD 2.5, the full-size
target on the thesis profile, 2.0 kg, a 4 m start, the target not held. What every archived
spec was flown with; analytic.launch.py and gazebo.launch.py take sitl.launch.py's defaults
instead. Arguments: sim_stack.py.

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
    return _stack().build(defaults=_stack().THESIS, bag_prefix='ibvs_')
