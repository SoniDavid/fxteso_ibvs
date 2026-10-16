"""FxTESO + ANFTIBVS with PX4 SITL as the plant; PX4 owns allocation, attitude and rates.
Start-up gates: px4_takeoff_gate starts the estimators, ibvs_gate starts pos_ctrl.

  ros2 launch quad_px4 sitl.launch.py [headless:=true] [rosbag:=true] [disturbance:=...]

Arguments and node builders live in px4_stack.py; see RUNNING_SIM.md.
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
    return _stack().build(mode='sitl')
