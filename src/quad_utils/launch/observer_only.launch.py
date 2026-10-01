"""Open loop: plant, scenario and the estimation chain, but no controllers.

The observer test - nothing can diverge from a control gain here. Expect /quad_thrust
and /quad_torques to stay silent. No ibvs_gate either: nothing is being held back, and the
recorder starts at launch. sitl.launch.py's defaults, as sim.launch.py; the fixed-time sweeps
flown at the thesis conditions pass them explicitly (sim_stack.py's THESIS).

  ros2 launch quad_utils observer_only.launch.py [plant:=gazebo] [disturbance:=gust]
                                                 [rosbag:=true]
                                                 [initial_estimate_offset:="[0.0,0.0,-0.5,0.0]"]
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
    return _stack().build(mode='observer', bag_prefix='obs_')
