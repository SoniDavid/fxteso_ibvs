"""Open loop: plant, scenario and the estimation chain, but no controllers.

The observer test - nothing can diverge from a control gain here. Expect /quad_thrust
and /quad_torques to stay silent. No ibvs_gate either: nothing is being held back.

  ros2 launch quad_utils observer_only.launch.py [plant:=gazebo] [disturbance:=gust]
                                                 [rosbag:=true]
                                                 [initial_estimate_offset:="[0.0,0.0,-0.5,0.0]"]
"""
import importlib.util
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = 'quad_utils'
SIM_PKG = 'quad_gz_sim'
CTRL_PKG = 'quad_control'


def bag_topics():
    """Same grouping sim.launch.py records; see quad_utils/config/bag_topics.yaml."""
    path = os.path.join(get_package_share_directory(PKG), 'config', 'bag_topics.yaml')
    with open(path) as fh:
        groups = yaml.safe_load(fh)
    return [topic for group in groups.values() for topic in group]


def include(pkg, name, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=(launch_arguments or {}).items())


def camera_presets():
    """quad_gz_sim's camera preset resolver - the one implementation of the intrinsics and
    the aD derivation. share/<pkg>/launch is not on sys.path, so it is loaded by path."""
    path = os.path.join(get_package_share_directory(SIM_PKG), 'launch', 'camera_presets.py')
    spec = importlib.util.spec_from_file_location('camera_presets', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    presets = camera_presets()
    args = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value='ibvs'),
        DeclareLaunchArgument('plant', default_value='analytic'),
        DeclareLaunchArgument('unwrap_attitude', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('disturbance', default_value='none'),
        DeclareLaunchArgument('disturbance_seed', default_value='0'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        # The seeded initial estimation error, forwarded to fixed_eso. This launch file is the
        # vehicle for the fixed-time convergence sweep, so it is the one that needs it.
        DeclareLaunchArgument('initial_estimate_offset',
                              default_value='[0.0, 0.0, 0.0, 0.0]'),
        # Target trajectory; see quad_gz_sim/scenario.launch.py.
        DeclareLaunchArgument('target_profile', default_value='thesis'),
        DeclareLaunchArgument('target_speed', default_value='1.0'),
        DeclareLaunchArgument('target_yaw_rate', default_value='0.1'),
        DeclareLaunchArgument('target_accel', default_value='0.5'),
        # Above zero, places fixed_eso's x/y gains as a triple pole at this rate.
        DeclareLaunchArgument('observer_omega', default_value='0.0'),
        # plant:=analytic only. 2.5 matches zD, i.e. px4's post-takeoff estimation error.
        DeclareLaunchArgument('start_altitude', default_value='4.0'),
        # Same camera plumbing as sim.launch.py: one preset feeds both the rendered <camera>
        # block and image_features, or the observer runs against a camera that was not rendered.
        DeclareLaunchArgument('camera', default_value=presets.DEFAULT_CAMERA),
        DeclareLaunchArgument('target_scale', default_value='1.0'),
        DeclareLaunchArgument('marker_dict', default_value='7x7'),
        DeclareLaunchArgument('camera_rate', default_value='0.0'),
        DeclareLaunchArgument('gust_scale', default_value='1.0'),
        DeclareLaunchArgument('wind_scale', default_value='1.0'),
        DeclareLaunchArgument('gust_tau', default_value='1.5'),
        # table52 only: scales the Von Karman sigmas, leaving the mean schedule alone.
        DeclareLaunchArgument('turbulence_scale', default_value='1.0'),
        DeclareLaunchArgument('zD', default_value='2.5'),
    ]

    def _estimation(context, *a, **k):
        cam = presets.resolve(
            LaunchConfiguration('camera').perform(context),
            float(LaunchConfiguration('target_scale').perform(context)),
            float(LaunchConfiguration('zD').perform(context)))
        return [
            LogInfo(msg=presets.summary(
                cam, float(LaunchConfiguration('zD').perform(context)),
                float(LaunchConfiguration('camera_rate').perform(context)))),
            include(CTRL_PKG, 'estimation.launch.py',
                    {'initial_estimate_offset':
                         LaunchConfiguration('initial_estimate_offset'),
                     'observer_omega': LaunchConfiguration('observer_omega'),
                     'camera_hfov': '%.9f' % cam['hfov'],
                     'camera_width': str(cam['width']),
                     'camera_height': str(cam['height']),
                     'camera_distortion': str(cam['distortion']),
                     'aD': '%.12g' % cam['aD'],
                     'marker_dict': LaunchConfiguration('marker_dict'),
                     'camera_topic': ('/quad/camera/image_distorted'
                                      if any(cam['distortion'])
                                      else '/quad/camera/image_raw')}),
        ]

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        # mcap, not sqlite3: Foxglove Studio cannot open .db3. Same convention as sim.launch.py.
        out = os.path.join('bags', 'obs_' + time.strftime('%Y%m%d_%H%M%S'))
        return [ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out] + bag_topics(),
            output='screen')]

    return LaunchDescription(args + [
        include(SIM_PKG, 'gz_sim.launch.py',
                {'headless': LaunchConfiguration('headless'),
                 'world': LaunchConfiguration('world'),
                 'plant': LaunchConfiguration('plant'),
                 'camera': LaunchConfiguration('camera'),
                 'target_scale': LaunchConfiguration('target_scale'),
                 'marker_dict': LaunchConfiguration('marker_dict'),
                 'camera_rate': LaunchConfiguration('camera_rate')}),
        include(SIM_PKG, 'plant.launch.py',
                {'plant': LaunchConfiguration('plant'),
                 'unwrap_attitude': LaunchConfiguration('unwrap_attitude'),
                 'start_altitude': LaunchConfiguration('start_altitude')}),
        include(SIM_PKG, 'scenario.launch.py',
                {'disturbance': LaunchConfiguration('disturbance'),
                 'disturbance_seed': LaunchConfiguration('disturbance_seed'),
                 'gust_scale': LaunchConfiguration('gust_scale'),
                 'wind_scale': LaunchConfiguration('wind_scale'),
                 'gust_tau': LaunchConfiguration('gust_tau'),
                 'turbulence_scale': LaunchConfiguration('turbulence_scale'),
                 # scenario measures /position_error against this and evaluates table52's
                 # altitude-dependent sigmas at it. Its own default is 2.5, so without this
                 # every non-2.5 depth carries a constant bias in that error.
                 'zD': LaunchConfiguration('zD'),
                 'target_profile': LaunchConfiguration('target_profile'),
                 'target_speed': LaunchConfiguration('target_speed'),
                 'target_yaw_rate': LaunchConfiguration('target_yaw_rate'),
                 'target_accel': LaunchConfiguration('target_accel')}),
        OpaqueFunction(function=_estimation),
        include(PKG, 'viz.launch.py', {'foxglove': LaunchConfiguration('foxglove')}),
        OpaqueFunction(function=_bag),
    ])
