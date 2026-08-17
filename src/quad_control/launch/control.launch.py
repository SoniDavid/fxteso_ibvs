"""The two control loops: adaptive-gain SMC on the IBVS error (50 Hz), then on
attitude (100 Hz).

No gating logic on purpose. Starting these before the plant, the target and a marker
lock are up commands the aircraft off garbage estimates, so the caller holds them back -
quad_utils/launch/sim.launch.py waits on ibvs_gate's exit code.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

PKG = 'quad_control'

NODES = [
    ('pos_ctrl', True),
    ('att_ctrl', False),
]


def generate_launch_description():
    return LaunchDescription([
        Node(package=PKG, executable=exe, name=exe,
             output='screen' if verbose else 'log',
             parameters=[{'use_sim_time': True}])
        for exe, verbose in NODES
    ])
