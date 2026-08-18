"""FxTESO + ANFTIBVS on top of PX4 SITL. The full stack with PX4 as the plant.

  quad_gz_sim/gz_sim (plant:=px4) -> world, ArUco target, camera bridge, /clock
        |                                              |
        |                            PX4 SITL (standalone) attaches to the F450
        |                                   already in that world
        |                                              |
        |                              MicroXRCEAgent  <-> uXRCE-DDS
        v                                              |
  quad_gz_sim/scenario -> tgt_* and /disturbances      v
                                              px4_state_adapter
                                    quad_position / quad_attitude / ...
                                                       |
  quad_control/estimation: td_linear, td_attitude, td_attitude_desired, image_features
                                                       |
                        fixed_eso -> pos_ctrl -> px4_offboard_bridge -> /fmu/in/*

PX4 owns allocation, attitude and rates, so att_ctrl does not run here. Everything from
image_features through pos_ctrl is the same code the analytic and gazebo plants use.

Start-up runs through two gates:

  px4_takeoff_gate   aircraft settled at the servoing altitude -> start the estimators
  ibvs_gate          plant, target and a held marker lock      -> start pos_ctrl

  ros2 launch quad_px4 sitl.launch.py [headless:=true] [rosbag:=true] [foxglove:=true]
                                      [controllers:=false]
                                      [disturbance:=none|step|gust|wind|csv]
                                      [disturbance_seed:=N]
                                      [px4_dir:=...] [xrce_agent:=...]

Prerequisites, both one-off:
  git submodule update --init --recursive external/PX4-Autopilot
  make -C external/PX4-Autopilot px4_sitl_default
"""
import math
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'quad_px4'
SIM_PKG = 'quad_gz_sim'
CTRL_PKG = 'quad_control'
UTILS_PKG = 'quad_utils'

# Must match <world name=...> in quad_gz_sim/worlds/ibvs.sdf and the <name> of the F450
# include in it: PX4 addresses both by name when it attaches.
WORLD = 'ibvs'
MODEL = 'F450'
# The airframe id of ROMFS/px4fmu_common/init.d-posix/airframes/22100_gz_F450_px4 in the
# PX4 fork. 22100 sits in PX4's reserved [22000, 22999] custom-model range.
SYS_AUTOSTART = '22100'

# Spawn pose from worlds/ibvs.sdf, in NED (the world is ENU, so y and z are negated). EKF2
# anchors its local frame there; px4_state_adapter adds this to get back to the world frame.
SPAWN_NED = (-9.9, -10.1, -0.1)

# EKF2's North is Gazebo +y (the world is ENU) while the workspace calls +x North.
FRAME_YAW_OFFSET = -math.pi / 2.0


def include(pkg, name, launch_arguments=None):
    """Include a sibling package's layer launch file."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=(launch_arguments or {}).items())


def bag_topics():
    """Flatten quad_utils' config/bag_topics.yaml, which groups topics by their source."""
    path = os.path.join(get_package_share_directory(UTILS_PKG), 'config', 'bag_topics.yaml')
    with open(path) as fh:
        groups = yaml.safe_load(fh)
    return [topic for group in groups.values() for topic in group]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('controllers', default_value='true'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('disturbance', default_value='none'),
        DeclareLaunchArgument('disturbance_seed', default_value='0'),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.path.expanduser(
                '~/Robotics/fxteso_ibvs/external/PX4-Autopilot'),
            description='The PX4 fork submodule, holding build/px4_sitl_default.'),
        DeclareLaunchArgument(
            'xrce_agent',
            default_value=os.path.expanduser(
                '~/Robotics/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'),
            description='MicroXRCEAgent binary. It is not normally on PATH.'),
        # Anchors the newton -> normalized thrust map; keep equal to MPC_THR_HOVER in the
        # fork's airframes/22100_gz_F450_px4, which documents how it was measured.
        DeclareLaunchArgument('hover_thrust', default_value='0.716'),
    ]

    simulation = [
        include(SIM_PKG, 'gz_sim.launch.py',
                {'headless': LaunchConfiguration('headless'),
                 'world': WORLD,
                 'plant': 'px4'}),
        include(SIM_PKG, 'scenario.launch.py',
                {'disturbance': LaunchConfiguration('disturbance'),
                 'disturbance_seed': LaunchConfiguration('disturbance_seed'),
                 # Arming and takeoff cost sim time the other plants do not spend; without
                 # this the target leaves the camera footprint before ibvs_gate can lock.
                 'hold_target': 'true'}),
    ]

    def _px4(context, *a, **k):
        """PX4 SITL in standalone mode, plus the uXRCE-DDS agent that fronts it for ROS 2."""
        px4_dir = LaunchConfiguration('px4_dir').perform(context)
        agent = LaunchConfiguration('xrce_agent').perform(context)

        binary = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
        rootfs = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs')
        if not os.path.isfile(binary):
            raise RuntimeError(
                '%s not found. Build PX4 first:  make -C %s px4_sitl_default' % (binary, px4_dir))
        if not os.path.isfile(agent):
            raise RuntimeError(
                '%s not found. Pass xrce_agent:=<path to MicroXRCEAgent>.' % agent)

        # STANDALONE stops PX4 starting its own gz server; MODEL_NAME binds it to the F450
        # this world already contains. See PX4's init.d-posix/px4-rc.gzsim.
        env = dict(os.environ)
        env.update({
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': WORLD,
            'PX4_GZ_MODEL_NAME': MODEL,
            'PX4_SYS_AUTOSTART': SYS_AUTOSTART,
            'PX4_GZ_NO_FOLLOW': '1',   # the world's own camera pose is the framing we want
        })

        return [
            ExecuteProcess(cmd=[agent, 'udp4', '-p', '8888'],
                           name='micro_xrce_agent', output='log'),
            # px4-rc.gzsim already polls 30 s for the world; this just keeps the console clean.
            TimerAction(period=5.0, actions=[
                ExecuteProcess(cmd=[binary], cwd=rootfs, env=env,
                               name='px4_sitl', output='screen'),
            ]),
        ]

    state_adapter = Node(
        package=PKG, executable='px4_state_adapter', name='px4_state_adapter',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'origin_north': SPAWN_NED[0],
                     'origin_east': SPAWN_NED[1],
                     'origin_down': SPAWN_NED[2],
                     'frame_yaw_offset': FRAME_YAW_OFFSET}])

    offboard_bridge = Node(
        package=PKG, executable='px4_offboard_bridge', name='px4_offboard_bridge',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'hover_thrust': LaunchConfiguration('hover_thrust'),
                     # Same value as the adapter's: one rotates into the workspace frame,
                     # the other rotates back out of it.
                     'frame_yaw_offset': FRAME_YAW_OFFSET}])

    viz = include(UTILS_PKG, 'viz.launch.py', {'foxglove': LaunchConfiguration('foxglove')})

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        # mcap, not rosbag2's default sqlite3: Foxglove Studio cannot open .db3.
        out = os.path.join('bags', 'px4_' + time.strftime('%Y%m%d_%H%M%S'))
        return [ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out] + bag_topics(),
            output='screen')]

    # Two gates in series: the aircraft has to be up before the estimators start (see
    # px4_takeoff_gate.cpp), and only then does the marker-lock gate make sense.
    #   px4_takeoff_gate -> estimation + ibvs_gate -> pos_ctrl
    takeoff_gate = Node(package=PKG, executable='px4_takeoff_gate', name='px4_takeoff_gate',
                        output='screen', parameters=[{'use_sim_time': True}])

    # Same gate as the other plants: exits 0 once the aircraft is placed, the target is
    # placed and the markers have held a lock.
    gate = Node(package=CTRL_PKG, executable='ibvs_gate', name='ibvs_gate',
                output='screen', parameters=[{'use_sim_time': True}])

    # pos_ctrl publishing desired_attitude is what tips px4_offboard_bridge into OFFBOARD,
    # so starting it here is the handover. att_ctrl stays off: PX4 owns the inner loop.
    control = RegisterEventHandler(OnProcessExit(
        target_action=gate,
        on_exit=lambda event, context: (
            ([include(CTRL_PKG, 'control.launch.py', {'attitude_controller': 'false'})]
             if LaunchConfiguration('controllers').perform(context).lower() == 'true'
             else [LogInfo(msg='controllers:=false - the aircraft will loiter after takeoff.')])
            + [OpaqueFunction(function=_bag)]
            if event.returncode == 0 else
            [LogInfo(msg='ibvs_gate failed - controllers not started. See its error above.')]
        )))

    estimation = RegisterEventHandler(OnProcessExit(
        target_action=takeoff_gate,
        on_exit=lambda event, context: (
            [include(CTRL_PKG, 'estimation.launch.py'), gate, control]
            if event.returncode == 0 else
            [LogInfo(msg='px4_takeoff_gate failed - estimators not started. See its error '
                         'above.')]
        )))

    return LaunchDescription(
        args + simulation
        + [OpaqueFunction(function=_px4), state_adapter, offboard_bridge, viz,
           estimation, takeoff_gate])
