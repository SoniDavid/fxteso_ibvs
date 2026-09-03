"""FxTESO + adaptive-gain SMC IBVS, ROS 2 Jazzy + Gazebo Harmonic. The full stack.

  quad_gz_sim/scenario  -> tgt_position/tgt_yaw/tgt_velocity/... and /disturbances
  quad_gz_sim/plant     -> quad_position/quad_attitude/...  (analytic or DART)
        |                                                   |
        |                        gz_pose_broadcaster --------+--> Gazebo (renderer only)
        |                                                          |
        |                                    [ros_gz_bridge] <-- /quad/camera/image_raw
        v                                                          |
  quad_control/estimation: td_linear / td_attitude / td_attitude_desired / image_features
        |                                                          |
        +---> fixed_eso ---> pos_ctrl ---> att_ctrl ---------------+--> quad_torques/quad_thrust

The plant is switchable; both backends publish the same five state topics.

  plant:=analytic  uav_dynamics.cpp integrates the aircraft; Gazebo is a camera only.
  plant:=gazebo    DART integrates the aircraft.

  ros2 launch quad_utils sim.launch.py [headless:=true] [rosbag:=true] [foxglove:=true]
                                       [plant:=analytic|gazebo] [controllers:=false]
                                       [disturbance:=none|step|gust|wind|table52|csv]
                                       [disturbance_seed:=N] [turbulence_scale:=1.0]
"""
import importlib.util
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'quad_utils'
SIM_PKG = 'quad_gz_sim'
CTRL_PKG = 'quad_control'


def camera_presets():
    """quad_gz_sim's camera preset resolver - the one implementation of the intrinsics and
    the aD derivation. share/<pkg>/launch is not on sys.path, so it is loaded by path."""
    path = os.path.join(get_package_share_directory(SIM_PKG), 'launch', 'camera_presets.py')
    spec = importlib.util.spec_from_file_location('camera_presets', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def include(pkg, name, launch_arguments=None):
    """Include a sibling package's layer launch file."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=(launch_arguments or {}).items())


def bag_topics():
    """Flatten config/bag_topics.yaml, which groups topics by where they come from."""
    path = os.path.join(get_package_share_directory(PKG), 'config', 'bag_topics.yaml')
    with open(path) as fh:
        groups = yaml.safe_load(fh)
    return [topic for group in groups.values() for topic in group]


def generate_launch_description():
    presets = camera_presets()
    args = [
        DeclareLaunchArgument('headless', default_value='false'),
        # Must match <world name=...> in quad_gz_sim/worlds/ibvs.sdf.
        DeclareLaunchArgument('world', default_value='ibvs'),
        # analytic = the paper's ROS-side integrator; gazebo = DART.
        DeclareLaunchArgument('plant', default_value='analytic'),
        # false leaves the controllers unstarted, for open-loop testing.
        DeclareLaunchArgument('controllers', default_value='true'),
        # Debug only: false feeds a wrapped attitude, as a quaternion source would.
        DeclareLaunchArgument('unwrap_attitude', default_value='true'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        # Comma-separated subset of none,step,gust,wind,csv.
        # Magnitudes: quad_gz_sim/config/disturbances.yaml.
        DeclareLaunchArgument('disturbance', default_value='none'),
        DeclareLaunchArgument('disturbance_seed', default_value='0'),
        # Scale the magnitudes in quad_gz_sim/config/disturbances.yaml; one number per sweep
        # point, with the YAML keeping the shape.
        DeclareLaunchArgument('gust_scale', default_value='1.0'),
        DeclareLaunchArgument('wind_scale', default_value='1.0'),
        DeclareLaunchArgument('gust_tau', default_value='1.5'),
        # table52 only: scales the Von Karman sigmas, leaving the mean schedule alone.
        DeclareLaunchArgument('turbulence_scale', default_value='1.0'),
        # Target trajectory; see quad_gz_sim/scenario.launch.py.
        DeclareLaunchArgument('target_profile', default_value='thesis'),
        DeclareLaunchArgument('target_speed', default_value='1.0'),
        DeclareLaunchArgument('target_yaw_rate', default_value='0.1'),
        DeclareLaunchArgument('target_accel', default_value='0.5'),
        # Above zero, places fixed_eso's x/y gains as a triple pole at this rate.
        DeclareLaunchArgument('observer_omega', default_value='0.0'),
        # plant:=analytic only. 2.5 matches zD, i.e. px4's post-takeoff estimation error.
        DeclareLaunchArgument('start_altitude', default_value='4.0'),
        # fixed_eso's x/y position injection. The thesis' 18; plant:=px4 flies 9.
        DeclareLaunchArgument('gamma1_xy', default_value='18.0'),
        # Table 5.3 has gamma2_xy 10, gamma3_xy 7, gamma3_yaw 7.
        DeclareLaunchArgument('gamma2_xy', default_value='20.0'),
        DeclareLaunchArgument('gamma3_xy', default_value='4.0'),
        DeclareLaunchArgument('gamma3_yaw', default_value='3.0'),
        DeclareLaunchArgument('gamma1_yaw', default_value='5.0'),
        DeclareLaunchArgument('gamma2_yaw', default_value='16.0'),
        DeclareLaunchArgument('alpha_yaw', default_value='0.75'),
        DeclareLaunchArgument('beta_yaw', default_value='1.2'),
        DeclareLaunchArgument('gamma4_yaw', default_value='0.001'),
        # Camera module and sensor mode; see quad_gz_sim/config/cameras.yaml. The preset feeds
        # both the rendered <camera> block and image_features' intrinsics, from one table.
        DeclareLaunchArgument('camera', default_value=presets.DEFAULT_CAMERA),
        DeclareLaunchArgument('target_scale', default_value='1.0'),
        DeclareLaunchArgument('marker_dict', default_value='7x7'),
        # Above zero, renders the camera at this rate instead of the SDF's 50, so the real
        # sensor mode's frame rate can be flown against the 50 Hz loop.
        DeclareLaunchArgument('camera_rate', default_value='0.0'),
        # Servoing depth. aD follows from it and from target_scale, so this one argument moves
        # the whole depth model; passing aD by hand is only for deliberate mismatches.
        DeclareLaunchArgument('zD', default_value='2.5'),
    ]

    simulation = [
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
    ]

    def _estimation(context, *a, **k):
        # The camera numbers have to be resolved, not substituted: image_features takes
        # intrinsics and aD, and aD is computed from the preset, target_scale and zD.
        cam = presets.resolve(
            LaunchConfiguration('camera').perform(context),
            float(LaunchConfiguration('target_scale').perform(context)),
            float(LaunchConfiguration('zD').perform(context)))
        return [
            LogInfo(msg=presets.summary(
                cam, float(LaunchConfiguration('zD').perform(context)),
                float(LaunchConfiguration('camera_rate').perform(context)))),
            include(CTRL_PKG, 'estimation.launch.py',
                    {'observer_omega': LaunchConfiguration('observer_omega'),
                     'gamma1_xy': LaunchConfiguration('gamma1_xy'),
                     'gamma2_xy': LaunchConfiguration('gamma2_xy'),
                     'gamma3_xy': LaunchConfiguration('gamma3_xy'),
                     'gamma3_yaw': LaunchConfiguration('gamma3_yaw'),
                     'gamma1_yaw': LaunchConfiguration('gamma1_yaw'),
                     'gamma2_yaw': LaunchConfiguration('gamma2_yaw'),
                     'alpha_yaw': LaunchConfiguration('alpha_yaw'),
                     'beta_yaw': LaunchConfiguration('beta_yaw'),
                     'gamma4_yaw': LaunchConfiguration('gamma4_yaw'),
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

    estimation = [
        OpaqueFunction(function=_estimation),
        include(PKG, 'viz.launch.py', {'foxglove': LaunchConfiguration('foxglove'),
                                       'camera': LaunchConfiguration('camera')}),
    ]

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        # mcap, not rosbag2's default sqlite3: Foxglove Studio cannot open .db3.
        out = os.path.join('bags', 'ibvs_' + time.strftime('%Y%m%d_%H%M%S'))
        return [ExecuteProcess(
            # See sitl.launch.py: the recorded topics carry no header stamps, so the
            # recorder's clock is the only time axis the analysis has.
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '-s', 'mcap', '-o', out]
                + bag_topics(),
            output='screen')]

    # Exits 0 once closed-loop servoing is possible, non-zero if it never is. A Node, not
    # an include: the event handler below has to hold the action object.
    gate = Node(package=CTRL_PKG, executable='ibvs_gate', name='ibvs_gate',
                output='screen', parameters=[{'use_sim_time': True}])

    # Do NOT go back to a TimerAction: that delay is wall clock while these run on sim
    # time. The recorder joins the controllers so the bag has no dead air at the front.
    control = RegisterEventHandler(OnProcessExit(
        target_action=gate,
        on_exit=lambda event, context: (
            ([include(CTRL_PKG, 'control.launch.py',
                      {'zD': LaunchConfiguration('zD')})]
             if LaunchConfiguration('controllers').perform(context).lower() == 'true'
             else [LogInfo(msg='controllers:=false - plant left open-loop.')])
            + [OpaqueFunction(function=_bag)]
            if event.returncode == 0 else
            [LogInfo(msg='ibvs_gate failed - controllers not started. See its error above.')]
        )))

    return LaunchDescription(args + simulation + estimation + [control, gate])
