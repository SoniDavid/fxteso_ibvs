"""Props-off bench test: hardware.launch.py with the table-test arguments already set. Never for flight.
No Vicon; the takeoff and handover altitude checks are opened up so the aircraft can sit on the table.

  ros2 launch quad_px4 table_test.launch.py frame_yaw_offset:=<minus EKF2 heading>

frame_yaw_offset is minus `heading` from /fmu/out/vehicle_local_position_v1, re-read whenever the
aircraft is rotated. Any hardware.launch.py argument can still be passed, presets included.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PRESETS = {
    'agent_device': '/dev/ttyUSB0',
    'target_layout': 'single_marker',
    'consent_rc_aux': '1',
    'takeoff_alt': '0.0',
    'takeoff_tolerance': '100.0',
    'handover_tolerance': '100.0',
    'gate_timeout': '600.0',
    'bag_prefix': 'table',
    'vicon': 'false',
}
# No default: these are measured on site, and a stale value commands a yaw step at handover.
REQUIRED = {
    'frame_yaw_offset': 'minus the EKF2 heading, read with the aircraft where it now sits',
}


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in PRESETS.items()]
    args += [DeclareLaunchArgument(k, description=d) for k, d in REQUIRED.items()]
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('quad_px4'), 'launch', 'hardware.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in [*PRESETS, *REQUIRED]}.items())
    return LaunchDescription(args + [hardware])
