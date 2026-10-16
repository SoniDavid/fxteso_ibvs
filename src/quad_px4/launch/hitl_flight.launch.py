"""HITL flight: the companion computer's stack (the SITL stack minus PX4) against the real FC,
gated by hitl_wait. Same arguments as hitl_world.launch.py. See RUNNING_HITL.md.

  ros2 launch quad_px4 hitl_flight.launch.py [agent:=false] [pilot:=sim|rc] [rosbag:=true] [...]
"""
import importlib.util
import os


def _stack():
    """px4_stack.py beside this file; launch/ is not on sys.path, so it is loaded by path."""
    path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'px4_stack.py')
    spec = importlib.util.spec_from_file_location('px4_stack', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    return _stack().build(mode='hitl_flight')
