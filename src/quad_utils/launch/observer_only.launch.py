"""Open loop: plant, scenario and the estimation chain, but no controllers.

The observer test - nothing can diverge from a control gain here. Expect /quad_thrust
and /quad_torques to stay silent. No ibvs_gate either: nothing is being held back.

  ros2 launch quad_utils observer_only.launch.py [plant:=gazebo] [disturbance:=gust]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = 'quad_utils'
SIM_PKG = 'quad_gz_sim'
CTRL_PKG = 'quad_control'


def include(pkg, name, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=(launch_arguments or {}).items())


def generate_launch_description():
    args = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value='ibvs'),
        DeclareLaunchArgument('plant', default_value='analytic'),
        DeclareLaunchArgument('unwrap_attitude', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('disturbance', default_value='none'),
        DeclareLaunchArgument('disturbance_seed', default_value='0'),
    ]

    return LaunchDescription(args + [
        include(SIM_PKG, 'gz_sim.launch.py',
                {'headless': LaunchConfiguration('headless'),
                 'world': LaunchConfiguration('world'),
                 'plant': LaunchConfiguration('plant')}),
        include(SIM_PKG, 'plant.launch.py',
                {'plant': LaunchConfiguration('plant'),
                 'unwrap_attitude': LaunchConfiguration('unwrap_attitude')}),
        include(SIM_PKG, 'scenario.launch.py',
                {'disturbance': LaunchConfiguration('disturbance'),
                 'disturbance_seed': LaunchConfiguration('disturbance_seed')}),
        include(CTRL_PKG, 'estimation.launch.py'),
        include(PKG, 'viz.launch.py', {'foxglove': LaunchConfiguration('foxglove')}),
    ])
