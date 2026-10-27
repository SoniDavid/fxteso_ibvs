"""Flight: hardware.launch.py with Vicon aiding EKF2 for the whole flight (full pose, never the
controller; velocity_source stays off). Real takeoff and handover altitude checks.

  ros2 launch quad_px4 flight.launch.py vicon_yaw_offset:=<rad> hover_thrust:=<MPC_THR_HOVER>

The flight controller must carry params/hardware.params and then params/hardware_vicon.params,
followed by a reboot; the Vicon receiver on the laptop must export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA.
Any hardware.launch.py argument can still be passed, presets included.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PRESETS = {
    'vicon': 'true',
    'aiding_policy': 'always',
    'target_layout': 'single_marker',
    'consent_rc_aux': '1',
    'bag_prefix': 'flight',
    # EKF2's heading is the Vicon frame here, so the datum lives in vicon_yaw_offset.
    'frame_yaw_offset': '0.0',
    # The 3S F450 as flown, battery in; hardware.launch's 2.0 is the thesis vehicle's.
    'quad_mass': '1.30',
}
# No default: measurements, never inherited. hardware.launch's 0.60 is a placeholder.
REQUIRED = {
    'vicon_yaw_offset': 'Vicon world onto the workspace datum (the printed target), rad',
    'hover_thrust': 'MPC_THR_HOVER read off a real hover',
}


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in PRESETS.items()]
    args += [DeclareLaunchArgument(k, description=d) for k, d in REQUIRED.items()]
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('quad_px4'), 'launch', 'hardware.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in [*PRESETS, *REQUIRED]}.items())
    return LaunchDescription(args + [hardware])
