"""The two control loops: adaptive-gain SMC on the IBVS error (50 Hz), then on
attitude (100 Hz).

No gating logic on purpose. Starting these before the plant, the target and a marker
lock are up commands the aircraft off garbage estimates, so the caller holds them back -
quad_utils/launch/sim.launch.py waits on ibvs_gate's exit code.

  attitude_controller:=false  drops att_ctrl and leaves only the IBVS loop. That is what
  plant:=px4 wants: PX4's mc_att_control and mc_rate_control take the inner loop, and
  quad_px4's offboard bridge consumes pos_ctrl's desired_attitude directly.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'quad_control'


def generate_launch_description():
    args = [
        DeclareLaunchArgument('attitude_controller', default_value='true'),
        # Servoing depth. 2.5 is the value pos_ctrl carried hardcoded and every recorded bag
        # was flown at; a smaller one is what a low lab ceiling needs. It has to move together
        # with image_features' aD and with the plant's takeoff altitude - the caller owns that.
        DeclareLaunchArgument('zD', default_value='2.5'),
        # False on hardware: SimRate blocks on node->now(), so with no /clock this hangs.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # The same number estimation.launch.py gives fixed_eso; see there.
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        # Which velocity feeds pos_ctrl's Coriolis term: td (default, what every recorded bag
        # was flown with) | ekf2 | vision | off. See aibvs_pos_ctrl.cpp and e83.md - indoors
        # the td path carries EKF2's dead-reckoned position, differentiated.
        DeclareLaunchArgument('velocity_source', default_value='td',
                              choices=['td', 'ekf2', 'vision', 'off']),
        # pos_ctrl's per-cycle error print. Off by default: at 50 Hz it buried every WARN in the
        # launch console, including the lock losses and PX4 failing safe. The same numbers are on
        # /error_visual_servoing.
        DeclareLaunchArgument('print_error', default_value='false'),
    ]

    def _nodes(context, *a, **k):
        # (executable, verbose)
        nodes = [('pos_ctrl', True)]
        if LaunchConfiguration('attitude_controller').perform(context).lower() == 'true':
            nodes.append(('att_ctrl', False))

        def params(exe):
            # Only pos_ctrl declares zD; handing it to att_ctrl would fail its launch.
            p = {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}
            if exe == 'pos_ctrl':
                p['zD'] = ParameterValue(LaunchConfiguration('zD'), value_type=float)
                p['quad_mass'] = ParameterValue(
                    LaunchConfiguration('quad_mass'), value_type=float)
                p['velocity_source'] = ParameterValue(
                    LaunchConfiguration('velocity_source'), value_type=str)
                p['print_error'] = ParameterValue(
                    LaunchConfiguration('print_error'), value_type=bool)
            return [p]

        return [
            Node(package=PKG, executable=exe, name=exe,
                 output='screen' if verbose else 'log',
                 parameters=params(exe))
            for exe, verbose in nodes
        ]

    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
