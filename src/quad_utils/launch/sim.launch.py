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
                                       [disturbance:=none|step|gust|wind|csv]
                                       [disturbance_seed:=N]
"""
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
    ]

    simulation = [
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
    ]

    estimation = [
        include(CTRL_PKG, 'estimation.launch.py'),
        include(PKG, 'viz.launch.py', {'foxglove': LaunchConfiguration('foxglove')}),
    ]

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        # mcap, not rosbag2's default sqlite3: Foxglove Studio cannot open .db3.
        out = os.path.join('bags', 'ibvs_' + time.strftime('%Y%m%d_%H%M%S'))
        return [ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out] + bag_topics(),
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
            ([include(CTRL_PKG, 'control.launch.py')]
             if LaunchConfiguration('controllers').perform(context).lower() == 'true'
             else [LogInfo(msg='controllers:=false - plant left open-loop.')])
            + [OpaqueFunction(function=_bag)]
            if event.returncode == 0 else
            [LogInfo(msg='ibvs_gate failed - controllers not started. See its error above.')]
        )))

    return LaunchDescription(args + simulation + estimation + [control, gate])
