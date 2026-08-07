"""FxTESO + adaptive-gain SMC IBVS, ROS 2 Jazzy + Gazebo Harmonic.

  target_position -> tgt_position/tgt_yaw/tgt_velocity/...
  uav_dynamics    -> quad_position/quad_attitude/... (the plant, integrated in ROS at 100 Hz)
        |                                                   |
        |                        gz_pose_broadcaster --------+--> Gazebo (renderer only)
        |                                                          |
        |                                    [ros_gz_bridge] <-- /quad/camera/image_raw
        v                                                          |
  td_linear / td_attitude / td_attitude_desired            image_features
        |                                                          |
        +---> fixed_eso ---> pos_ctrl ---> att_ctrl ---------------+--> quad_torques/quad_thrust

Gazebo is a camera and nothing else: zero gravity, both model poses overwritten every 10 ms
from the ROS side. See documentation.md.

  ros2 launch fxteso_ibvs sim.launch.py [headless:=true] [rosbag:=true] [foxglove:=true]
                                        [disturbance:=none|step|gust|wind|csv]
                                        [disturbance_seed:=N]
"""
import os
import signal
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'fxteso_ibvs'

# Camera images excluded: 820x616 at 50 Hz is ~75 MB/s and would dwarf every signal here.
BAG_TOPICS = [
    # plant and target
    '/quad_position', '/quad_attitude', '/quad_velocity', '/quad_velocity_BF',
    '/quad_attitude_velocity', '/quad_thrust', '/quad_torques', '/desired_attitude',
    '/tgt_position', '/tgt_velocity', '/tgt_yaw', '/disturbances',
    # vision and the fixed-time observer (thesis Figs 5.10d, 5.11a, 5.12)
    '/ImFeat_vector', '/ImFeat_estimates_fxt', '/ImFeat_dot_estimates_fxt',
    '/estimation_error_fxt', '/ibvs_dist', '/scaled_ibvs_dist', '/a_value', '/z_des',
    '/u_coordinates', '/n_coordinates',
    # position-loop control internals (Figs 5.13, 5.14a)
    '/error_visual_servoing', '/error_dot_visual_servoing', '/ibvs_ss',
    '/ibvs_control_input', '/adaptive_gain', '/position_error',
    # attitude-loop control internals
    '/error_att', '/error_attVel', '/sigma_att', '/k_att',
    # tracking differentiators and their estimation errors (Fig 5.11b)
    '/attitude_estimates', '/attitude_desired_estimates', '/lin_vel_BF_estimates',
    '/position_estimates', '/attitude_td_error', '/attitude_des_td_error',
    '/position_td_error',
    # Foxglove 3D panel
    '/tf', '/quad_path', '/tgt_path', '/ibvs_markers',
]

# uav_dynamics holds its initial condition for 5 s before integrating, so these all need
# that head start.
PLANT_NODES = [
    ('uav_dynamics',         False),
    ('disturbances',         True),   # logs the resolved profile at startup
    ('target_position',      False),
    ('image_features',       True),   # prints "I see N arucos only" when it loses lock
    ('td_attitude',          False),
    ('td_attitude_desired',  False),
    ('td_linear',            False),
    ('fixed_eso',            False),
    ('tf_broadcaster',       False),  # TF + paths + markers, for Foxglove's 3D panel
]

# Held back until ibvs_gate says the simulation is ready. Do NOT go back to a TimerAction:
# that delay is wall clock while these nodes run on sim time. See src/ibvs_gate.cpp.
CTRL_NODES = [
    ('pos_ctrl',             True),
    ('att_ctrl',             False),
]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    world_file = os.path.join(share, 'worlds', 'ibvs.sdf')
    bridge_cfg = os.path.join(share, 'config', 'bridge.yaml')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    args = [
        DeclareLaunchArgument('headless', default_value='false'),
        # Must match <world name=...> in worlds/ibvs.sdf: selects the gz topic
        # /world/<world>/set_pose_vector the broadcaster publishes on.
        DeclareLaunchArgument('world', default_value='ibvs'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        # Comma-separated subset of none,step,gust,wind,csv. Magnitudes: config/disturbances.yaml.
        DeclareLaunchArgument('disturbance', default_value='none'),
        DeclareLaunchArgument('disturbance_seed', default_value='0'),
    ]

    # The YAML omits profile/seed so these two never silently lose to it. value_type is
    # required: a LaunchConfiguration is a string, declare_parameter<int> is not.
    disturbance_params = [
        os.path.join(share, 'config', 'disturbances.yaml'),
        {'profile': ParameterValue(LaunchConfiguration('disturbance'), value_type=str),
         'seed': ParameterValue(LaunchConfiguration('disturbance_seed'), value_type=int)},
    ]

    # Resolves model://F450 and model://aruco_target.
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models'))

    # Without this, gz-sim's sensor rendering picks the Mesa ICD and falls back to software
    # (llvmpipe) even though the GPU is present.
    egl_vendor = [
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                               '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
    ]

    def _reap_orphans(context, *a, **k):
        """Kill gz sim processes orphaned by a previous run (they hold a GPU context).

        Requires all three of: 'gz sim' in the cmdline, this package's name in it, and
        parent pid 1 - so a live simulation, or a Gazebo running for anything else, is
        never touched.
        """
        killed = []
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            try:
                with open('/proc/%s/cmdline' % entry, 'rb') as fh:
                    cmd = fh.read().decode('utf-8', 'replace').replace('\0', ' ')
                if 'gz sim' not in cmd or PKG not in cmd:
                    continue
                # "pid (comm) state ppid ..."; comm may contain spaces, so split last ')'.
                with open('/proc/%s/stat' % entry) as fh:
                    ppid = int(fh.read().rsplit(')', 1)[1].split()[1])
                if ppid != 1:
                    continue
                os.kill(int(entry), signal.SIGKILL)
                killed.append(entry)
            except (OSError, ValueError, IndexError):
                continue

        if not killed:
            return []
        return [LogInfo(msg='Reaped orphaned gz sim from a previous run: pid ' +
                            ', '.join(killed))]

    def _gz(context, *a, **k):
        headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
        # -r starts the world running, so nothing has to unpause physics.
        flags = '-r -v3 -s --headless-rendering ' if headless else '-r -v3 '
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': flags + world_file, 'gz_version': '8'}.items(),
        )]

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ibvs_bridge',
        output='screen', parameters=[{'config_file': bridge_cfg}],
    )

    broadcaster = Node(
        package=PKG, executable='gz_pose_broadcaster', name='gz_pose_broadcaster',
        output='screen', parameters=[{'world': LaunchConfiguration('world'),
                                      'use_sim_time': True}],
    )

    # Every node runs on Gazebo's clock; /clock is bridged in config/bridge.yaml.
    def _node(exe, verbose, params=None):
        # params may mix YAML paths and dicts; later entries win.
        return Node(package=PKG, executable=exe, name=exe,
                    output='screen' if verbose else 'log',
                    parameters=[{'use_sim_time': True}] + list(params or []))

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        # mcap, not rosbag2's default sqlite3: Foxglove Studio cannot open .db3.
        out = os.path.join('bags', 'ibvs_' + time.strftime('%Y%m%d_%H%M%S'))
        return [ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out] + BAG_TOPICS,
            output='screen')]

    def _foxglove(context, *a, **k):
        if LaunchConfiguration('foxglove').perform(context).lower() != 'true':
            return []
        # foxglove_bridge_launch.xml is a frontend launch file, hence Any, not Python.
        return [IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch', 'foxglove_bridge_launch.xml')))]

    plant = [_node(exe, v, disturbance_params if exe == 'disturbances' else None)
             for exe, v in PLANT_NODES]

    # Exits 0 the moment closed-loop servoing is actually possible, and non-zero (leaving
    # the controllers unstarted, loudly) if that never happens.
    gate = _node('ibvs_gate', True)

    # The recorder joins the controllers so the bag has no dead air at the front.
    control = RegisterEventHandler(OnProcessExit(
        target_action=gate,
        on_exit=lambda event, context: (
            [_node(exe, v) for exe, v in CTRL_NODES] + [OpaqueFunction(function=_bag)]
            if event.returncode == 0 else
            [LogInfo(msg='ibvs_gate failed - controllers not started. See its error above.')]
        )))

    return LaunchDescription(args + egl_vendor + [
        resource_path,
        OpaqueFunction(function=_reap_orphans),   # before Gazebo starts
        OpaqueFunction(function=_gz),
        OpaqueFunction(function=_foxglove),
        bridge,
        broadcaster,
    ] + plant + [control, gate])
