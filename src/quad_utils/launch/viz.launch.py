"""TF, trajectory trails, markers, and optionally the Foxglove bridge.

Pure consumer, so it is safe to restart against a live sim or a bag replay:

  ros2 bag play bags/ibvs_20260817_120000 --clock
  ros2 launch quad_utils viz.launch.py foxglove:=true

Layout for Studio: quad_utils/foxglove/. There is more than one, because the Vision tab shows
what image_features actually consumes and that topic depends on the camera preset - so this
prints the path to import for the preset in use. Pass camera:= to get the right one.
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'quad_utils'


def _variants():
    """foxglove/variants.py, which owns the preset -> layout mapping. share/<pkg>/foxglove is
    not on sys.path, so it is loaded by path, as sitl.launch.py does with camera_presets."""
    path = os.path.join(get_package_share_directory(PKG), 'foxglove', 'variants.py')
    spec = importlib.util.spec_from_file_location('foxglove_variants', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    args = [DeclareLaunchArgument('foxglove', default_value='false'),
            # Only used to name the layout to import; the preset itself is quad_gz_sim's.
            DeclareLaunchArgument('camera', default_value='')]

    tf_broadcaster = Node(
        package=PKG, executable='tf_broadcaster', name='tf_broadcaster',
        output='log', parameters=[{'use_sim_time': True}])

    def _foxglove(context, *a, **k):
        if LaunchConfiguration('foxglove').perform(context).lower() != 'true':
            return []

        # Naming the layout is not decoration: importing the wrong variant leaves the Vision
        # tab blank, which looks exactly like a camera that is not publishing.
        camera = LaunchConfiguration('camera').perform(context)
        share = os.path.join(get_package_share_directory(PKG), 'foxglove')
        try:
            name = _variants().layout_for(camera) if camera else None
        except KeyError:
            name = None
        msg = ('Foxglove layout: %s' % os.path.join(share, name) if name else
               'Foxglove layout: %s (pass camera:= to pick the variant for your preset)'
               % share)

        # foxglove_bridge_launch.xml is a frontend launch file, hence Any, not Python.
        return [LogInfo(msg=msg), IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch', 'foxglove_bridge_launch.xml')))]

    return LaunchDescription(args + [tf_broadcaster,
                                     OpaqueFunction(function=_foxglove)])
