"""The moving target and the disturbance force.

  ros2 launch quad_gz_sim scenario.launch.py [disturbance:=none|step|gust|wind|csv]
                                             [disturbance_seed:=N]

disturbance takes a comma-separated subset; profiles compose. Magnitudes:
config/disturbances.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'quad_gz_sim'


def generate_launch_description():
    share = get_package_share_directory(PKG)

    args = [
        DeclareLaunchArgument('disturbance', default_value='none'),
        DeclareLaunchArgument('disturbance_seed', default_value='0'),
        # true holds the target on its start pose until pos_ctrl publishes. plant:=px4 needs
        # it, because arming and taking off cost sim time the other plants do not spend.
        DeclareLaunchArgument('hold_target', default_value='false'),
    ]

    # The YAML omits profile/seed so these two never silently lose to it. value_type is
    # required: a LaunchConfiguration is a string, declare_parameter<int> is not.
    disturbance_params = [
        os.path.join(share, 'config', 'disturbances.yaml'),
        {'use_sim_time': True,
         'profile': ParameterValue(LaunchConfiguration('disturbance'), value_type=str),
         'seed': ParameterValue(LaunchConfiguration('disturbance_seed'), value_type=int)},
    ]

    disturbances = Node(
        package=PKG, executable='disturbances', name='disturbances',
        output='screen',              # logs the resolved profile at startup
        parameters=disturbance_params)

    target_position = Node(
        package=PKG, executable='target_position', name='target_position',
        output='log',
        parameters=[{'use_sim_time': True,
                     'hold_until_control': ParameterValue(
                         LaunchConfiguration('hold_target'), value_type=bool)}])

    return LaunchDescription(args + [disturbances, target_position])
