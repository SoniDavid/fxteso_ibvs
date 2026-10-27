"""The two control loops: adaptive-gain SMC on the IBVS error (50 Hz), then on attitude (100 Hz).
No gating here; the caller starts it after ibvs_gate. attitude_controller:=false for plant:=px4.
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
        # Servoing depth; must move with image_features' aD and the takeoff altitude (the caller's job).
        DeclareLaunchArgument('zD', default_value='2.5'),
        # False on hardware: SimRate blocks on node->now(), so with no /clock this hangs.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # The same number estimation.launch.py gives fixed_eso; see there.
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        # Velocity in pos_ctrl's Coriolis term: td | ekf2 | vision | off (see aibvs_pos_ctrl.cpp).
        DeclareLaunchArgument('velocity_source', default_value='td',
                              choices=['td', 'ekf2', 'vision', 'off']),
        # pos_ctrl's per-cycle error print; /error_visual_servoing has the same numbers.
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
