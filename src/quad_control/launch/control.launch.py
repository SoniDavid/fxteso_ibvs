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
    ]

    def _nodes(context, *a, **k):
        # (executable, verbose)
        nodes = [('pos_ctrl', True)]
        if LaunchConfiguration('attitude_controller').perform(context).lower() == 'true':
            nodes.append(('att_ctrl', False))

        def params(exe):
            # Only pos_ctrl declares zD; handing it to att_ctrl would fail its launch.
            p = {'use_sim_time': True}
            if exe == 'pos_ctrl':
                p['zD'] = ParameterValue(LaunchConfiguration('zD'), value_type=float)
            return [p]

        return [
            Node(package=PKG, executable=exe, name=exe,
                 output='screen' if verbose else 'log',
                 parameters=params(exe))
            for exe, verbose in nodes
        ]

    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
