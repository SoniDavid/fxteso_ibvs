"""The estimation chain: camera and plant state in, feature and state estimates out.

Safe to run without the controllers - that is the open-loop observer test. Each node
staggers its own start on sim time, so nothing here needs a launch delay.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

PKG = 'quad_control'

NODES = [
    ('image_features',       True),   # prints "I see N arucos only" when it loses lock
    ('td_attitude',          False),
    ('td_attitude_desired',  False),
    ('td_linear',            False),
    ('fixed_eso',            False),
]


def generate_launch_description():
    # Every node runs on the simulator's clock; /clock is bridged in quad_gz_sim.
    return LaunchDescription([
        Node(package=PKG, executable=exe, name=exe,
             output='screen' if verbose else 'log',
             parameters=[{'use_sim_time': True}])
        for exe, verbose in NODES
    ])
