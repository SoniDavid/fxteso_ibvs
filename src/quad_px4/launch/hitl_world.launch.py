"""HITL world: Gazebo, the scenario, vicon_sim and hitl_bridge (USB to the real FC). Start it
first, then hitl_flight.launch.py with the same arguments. See RUNNING_HITL.md.

  ros2 launch quad_px4 hitl_world.launch.py [fc_device:=...|none] [min_rtf:=0.95] [...]
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
    return _stack().build(mode='hitl_world')
