"""TF, trajectory trails, markers, and optionally the Foxglove bridge.

Pure consumer, so it is safe to restart against a live sim or a bag replay:

  ros2 bag play bags/ibvs_20260817_120000 --clock
  ros2 launch quad_utils viz.launch.py foxglove:=true

Layout for Studio: quad_utils/foxglove/fxteso_ibvs.json.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'quad_utils'


def generate_launch_description():
    args = [DeclareLaunchArgument('foxglove', default_value='false')]

    tf_broadcaster = Node(
        package=PKG, executable='tf_broadcaster', name='tf_broadcaster',
        output='log', parameters=[{'use_sim_time': True}])

    def _foxglove(context, *a, **k):
        if LaunchConfiguration('foxglove').perform(context).lower() != 'true':
            return []
        # foxglove_bridge_launch.xml is a frontend launch file, hence Any, not Python.
        return [IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch', 'foxglove_bridge_launch.xml')))]

    return LaunchDescription(args + [tf_broadcaster,
                                     OpaqueFunction(function=_foxglove)])
