"""The analytic or gazebo stack by plant:=, with sitl.launch.py's defaults: zD 1.2 from a 1.5 m
start, the 0.5-scale target held until handover, the 1.30 kg 3S build. Identical to
analytic.launch.py and gazebo.launch.py apart from the plant argument.

Archived runs reproduce with sim_stack.py's THESIS set passed explicitly: zD:=2.5
target_scale:=1.0 target_profile:=thesis quad_mass:=2.0 start_altitude:=4.0 hold_target:=false
record_from:=handover sim_hold:=false.

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
