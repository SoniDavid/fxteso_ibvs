"""Vicon receiver for flight.launch.py, run on the laptop wired to the Vicon PC (the SDK is x86-64).
Publishes /vicon/<subject>/<segment> (fxteso_f450quad/fxteso_f450quad) to the Pi's vicon_px4_bridge.

  source ~/Robotics/ros2-vicon-jazzy/install/setup.bash
  ros2 launch quad_px4 vicon_receiver.launch.py

The Pi must be on the same subnet. LARGE_DATA matches hardware.launch: either side without it sees nothing.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # The lab's Vicon PC, found on the LAN by its DataStream port.
        DeclareLaunchArgument('vicon_host', default_value='192.168.1.23'),
        DeclareLaunchArgument('vicon_port', default_value='801'),
        DeclareLaunchArgument('buffer_size', default_value='200'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA'),
        Node(package='vicon_receiver', executable='vicon_client', name='vicon', output='screen',
             parameters=[{'hostname': LaunchConfiguration('vicon_host'),
                          'port': ParameterValue(LaunchConfiguration('vicon_port'), value_type=int),
                          'buffer_size': ParameterValue(LaunchConfiguration('buffer_size'),
                                                        value_type=int),
                          'namespace': 'vicon'}]),
    ])
