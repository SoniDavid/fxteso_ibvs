"""The moving target and the disturbance force.

  ros2 launch quad_gz_sim scenario.launch.py [disturbance:=none|step|gust|wind|table52|csv]
                                             [disturbance_seed:=N]
                                             [gust_scale:=1.0] [wind_scale:=1.0]

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
        # Multiply the magnitudes in config/disturbances.yaml, so a sweep varies one number
        # while the YAML keeps owning the shape of the disturbance.
        DeclareLaunchArgument('gust_scale', default_value='1.0'),
        DeclareLaunchArgument('wind_scale', default_value='1.0'),
        # table52 only: multiplies the Von Karman sigmas, leaving the mean schedule alone. 0
        # flies the thesis' wind STRUCTURE - the reversals and the downdraft - without gusting.
        DeclareLaunchArgument('turbulence_scale', default_value='1.0'),
        # The gust's own correlation time. Sweep it against the estimator lag; see
        # the gust-bandwidth sweep.
        DeclareLaunchArgument('gust_tau', default_value='1.5'),
        # true holds the target on its start pose until pos_ctrl publishes. plant:=px4 needs
        # it, because arming and taking off cost sim time the other plants do not spend.
        DeclareLaunchArgument('hold_target', default_value='false'),
        # Target trajectory. 'thesis' is what every recorded bag was flown on; the rest sweep
        # the Assumption 7 bounds. The target_ prefix keeps these clear of the disturbances
        # node's own 'profile' parameter, which is a different thing fed by 'disturbance'.
        DeclareLaunchArgument('target_profile', default_value='thesis',
                              description='thesis | hover | line | circle | steps'),
        DeclareLaunchArgument('target_speed', default_value='1.0',
                              description='m/s, used by line, circle and steps'),
        DeclareLaunchArgument('target_yaw_rate', default_value='0.1',
                              description='rad/s, used by circle'),
        DeclareLaunchArgument('target_accel', default_value='0.5',
                              description='m/s^2, used by line and steps'),
        # The servoing depth /position_error is measured against; must match the zD the
        # controller and aD are derived from, or that error carries a constant bias.
        DeclareLaunchArgument('zD', default_value='2.5'),
    ]

    # The YAML omits profile/seed so these two never silently lose to it. value_type is
    # required: a LaunchConfiguration is a string, declare_parameter<int> is not.
    disturbance_params = [
        os.path.join(share, 'config', 'disturbances.yaml'),
        {'use_sim_time': True,
         # Same gate the target uses: under px4 the aircraft is still on the ground 8 s in,
         # so a fixed hold lands the disturbance on a takeoff rather than on servoing.
         'hold_until_control': ParameterValue(
             LaunchConfiguration('hold_target'), value_type=bool),
         'profile': ParameterValue(LaunchConfiguration('disturbance'), value_type=str),
         'seed': ParameterValue(LaunchConfiguration('disturbance_seed'), value_type=int),
         'gust_scale': ParameterValue(LaunchConfiguration('gust_scale'), value_type=float),
         'wind_scale': ParameterValue(LaunchConfiguration('wind_scale'), value_type=float),
         'gust_tau': ParameterValue(LaunchConfiguration('gust_tau'), value_type=float),
         'turbulence_scale': ParameterValue(
             LaunchConfiguration('turbulence_scale'), value_type=float),
         # Eqs. 2.30-2.33 are altitude-dependent, so the thesis' own wind must be evaluated at
         # the depth actually being flown. Same zD the target and the controller use.
         'table52_altitude': ParameterValue(LaunchConfiguration('zD'), value_type=float)},
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
                         LaunchConfiguration('hold_target'), value_type=bool),
                     'profile': ParameterValue(
                         LaunchConfiguration('target_profile'), value_type=str),
                     'speed': ParameterValue(
                         LaunchConfiguration('target_speed'), value_type=float),
                     'yaw_rate': ParameterValue(
                         LaunchConfiguration('target_yaw_rate'), value_type=float),
                     'accel': ParameterValue(
                         LaunchConfiguration('target_accel'), value_type=float),
                     'zD': ParameterValue(
                         LaunchConfiguration('zD'), value_type=float)}])

    return LaunchDescription(args + [disturbances, target_position])
