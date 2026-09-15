"""The eleven-node flight subset on the real aircraft. No Gazebo, no PX4 SITL, no sim_pilot.

  ros2 launch quad_px4 hardware.launch.py

Differs from SITL in three ways that are not tuning: use_sim_time is false, frame_yaw_offset is
zero, and the camera runs at native resolution. att_ctrl stays off; PX4 owns the inner loop.

No camera driver is launched - there is none in this repo. Start it separately and point
camera_topic at it.
"""
import datetime
import importlib.util
import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'quad_px4'
CTRL_PKG = 'quad_control'
UTILS_PKG = 'quad_utils'
SIM_PKG = 'quad_gz_sim'   # for the camera preset table only; nothing simulated is launched

# The lab's Vicon coverage ceiling, not a tuning knob.
DEFAULT_ZD = 1.2
# Handover altitude. Above zD so the aircraft is settled, not still climbing.
DEFAULT_TAKEOFF_ALT = 1.5


def camera_presets():
    """quad_gz_sim's preset resolver. Loaded by path: share/<pkg>/launch is not on sys.path."""
    path = os.path.join(get_package_share_directory(SIM_PKG), 'launch', 'camera_presets.py')
    spec = importlib.util.spec_from_file_location('camera_presets', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def include(pkg, name, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=(launch_arguments or {}).items())


def bag_topics():
    """The same topic set SITL records, so the analysis scripts ingest a hardware bag."""
    path = os.path.join(get_package_share_directory(UTILS_PKG), 'config', 'bag_topics.yaml')
    with open(path) as fh:
        groups = yaml.safe_load(fh)
    return [topic for group in groups.values() for topic in group]


def generate_launch_description():
    args = [
        # --- the flight geometry ------------------------------------------------
        DeclareLaunchArgument('zD', default_value=str(DEFAULT_ZD),
                              description='servoing depth, m. The lab rig sets this.'),
        DeclareLaunchArgument('takeoff_alt', default_value=str(DEFAULT_TAKEOFF_ALT),
                              description='altitude the pilot flies to before the handover, m'),
        DeclareLaunchArgument('target_scale', default_value='1.0',
                              description='printed plate size relative to the reference target'),

        # --- the airframe -------------------------------------------------------
        # Weighed as flown, battery included. The 2.0 default is the simulated F450's.
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        # Must equal MPC_THR_HOVER on the aircraft; read it off a real hover.
        DeclareLaunchArgument('hover_thrust', default_value='0.60'),

        # --- the camera ---------------------------------------------------------
        DeclareLaunchArgument('camera', default_value='module3wide_2304'),
        # What the driver publishes. Empty means the preset's native mode.
        DeclareLaunchArgument('camera_width', default_value=''),
        DeclareLaunchArgument('camera_height', default_value=''),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        # True for a real driver: they publish BEST_EFFORT, and a RELIABLE
        # subscription to one receives nothing, silently.
        DeclareLaunchArgument('sensor_qos', default_value='true'),
        # Brown-Conrady k1,k2,p1,p2,k3 from a real calibration. The preset's are an assumption.
        DeclareLaunchArgument('camera_distortion', default_value='[0.0, 0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('marker_dict', default_value='7x7'),

        # --- the link -----------------------------------------------------------
        DeclareLaunchArgument('agent', default_value='true',
                              description='start MicroXRCEAgent here'),
        DeclareLaunchArgument('agent_device', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('agent_baud', default_value='921600'),

        # --- the handover -------------------------------------------------------
        # 1..6 selects manual_control_setpoint.auxN; needs RC_MAP_AUXn. 0 = service only.
        DeclareLaunchArgument('consent_rc_aux', default_value='0'),
        # Tighter than the node's 0.4 m default, which fires during the climb.
        DeclareLaunchArgument('handover_tolerance', default_value='0.15'),
        # ibvs_gate's alignment limit, tuned against a simulated pilot's perfect hover.
        DeclareLaunchArgument('max_feature_error', default_value='0.15'),
        DeclareLaunchArgument('gate_timeout', default_value='120.0'),
        DeclareLaunchArgument('estimators_ready', default_value='5.0'),
        # Barometer-only height. The SITL barometer is 300x quieter than PX4's own model.
        DeclareLaunchArgument('takeoff_tolerance', default_value='0.20'),

        # --- the workspace datum ------------------------------------------------
        # Zero, unlike SITL's -pi/2: that encodes the Gazebo world's ENU convention.
        DeclareLaunchArgument('frame_yaw_offset', default_value='0.0'),

        # --- recording ----------------------------------------------------------
        DeclareLaunchArgument('rosbag', default_value='true'),
        DeclareLaunchArgument('bag_prefix', default_value='hw'),
        DeclareLaunchArgument('foxglove', default_value='false'),
    ]

    def _camera(context):
        """Preset intrinsics at the resolution the driver actually publishes."""
        presets = camera_presets()
        cam = presets.resolve(
            LaunchConfiguration('camera').perform(context),
            float(LaunchConfiguration('target_scale').perform(context)),
            float(LaunchConfiguration('zD').perform(context)))
        w = LaunchConfiguration('camera_width').perform(context)
        h = LaunchConfiguration('camera_height').perform(context)
        # The preset's render size is a simulation speed choice; native is what a camera does.
        cam['width'] = int(w) if w else cam['native'][0]
        cam['height'] = int(h) if h else cam['native'][1]
        # fx follows the width; aD is metric and does not.
        cam['fx'] = cam['width'] / (2.0 * math.tan(cam['hfov'] / 2.0))
        return cam

    def _agent(context, *a, **k):
        if LaunchConfiguration('agent').perform(context).lower() != 'true':
            return []
        # Serial, not `udp4 -p 8888`: the flight controller is on a UART here, not a loopback.
        return [ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev',
                 LaunchConfiguration('agent_device'), '-b', LaunchConfiguration('agent_baud')],
            name='micro_xrce_agent', output='screen')]

    # EKF2 -> the five state topics. origin_* stay zero: no Gazebo world to reconcile with.
    state_adapter = Node(
        package=PKG, executable='px4_state_adapter', name='px4_state_adapter', output='screen',
        parameters=[{'use_sim_time': False,
                     'origin_north': 0.0, 'origin_east': 0.0, 'origin_down': 0.0,
                     'frame_yaw_offset': ParameterValue(
                         LaunchConfiguration('frame_yaw_offset'), value_type=float)}])

    # Holds the estimators until PX4 is settled at altitude. require_gnss false: cs_gnss_pos
    # never comes true indoors. Yaw alignment is still required.
    takeoff_gate = Node(
        package=PKG, executable='px4_takeoff_gate', name='px4_takeoff_gate', output='screen',
        parameters=[{'use_sim_time': False,
                     'altitude': ParameterValue(
                         LaunchConfiguration('takeoff_alt'), value_type=float),
                     'tolerance': ParameterValue(
                         LaunchConfiguration('takeoff_tolerance'), value_type=float),
                     'require_gnss': False}])

    # require_target false: tgt_position is the simulator's, and the target here is a plate.
    gate = Node(
        package=CTRL_PKG, executable='ibvs_gate', name='ibvs_gate', output='screen',
        parameters=[{'use_sim_time': False,
                     'max_feature_error': ParameterValue(
                         LaunchConfiguration('max_feature_error'), value_type=float),
                     'estimators_ready': ParameterValue(
                         LaunchConfiguration('estimators_ready'), value_type=float),
                     'timeout': ParameterValue(
                         LaunchConfiguration('gate_timeout'), value_type=float),
                     'require_target': False}])

    def _bridge(context, *a, **k):
        return [Node(
            package=PKG, executable='px4_offboard_bridge', name='px4_offboard_bridge',
            output='screen',
            parameters=[{'use_sim_time': False,
                         'hover_thrust': ParameterValue(
                             LaunchConfiguration('hover_thrust'), value_type=float),
                         'mass': ParameterValue(
                             LaunchConfiguration('quad_mass'), value_type=float),
                         'takeoff_altitude': ParameterValue(
                             LaunchConfiguration('takeoff_alt'), value_type=float),
                         # Explicit, not the node's 0.4 m default, which fires mid-climb.
                         'altitude_tolerance': ParameterValue(
                             LaunchConfiguration('handover_tolerance'), value_type=float),
                         # A human arms and flies it up. This never arms and never takes off.
                         'bringup': 'pilot',
                         # And a human says when. SITL passes false: sim_pilot cannot consent.
                         'require_consent': True,
                         'consent_rc_aux': ParameterValue(
                             LaunchConfiguration('consent_rc_aux'), value_type=int),
                         # Known harmful: it defeats the offboard-loss failsafe.
                         'offboard_recovery': False,
                         'frame_yaw_offset': ParameterValue(
                             LaunchConfiguration('frame_yaw_offset'), value_type=float)}])]

    def _estimation(context, *a, **k):
        cam = _camera(context)
        presets = camera_presets()
        return [
            LogInfo(msg=presets.summary(cam, float(LaunchConfiguration('zD').perform(context)))),
            LogInfo(msg='hardware: use_sim_time FALSE, frame_yaw_offset %s, camera %dx%d'
                        % (LaunchConfiguration('frame_yaw_offset').perform(context),
                           cam['width'], cam['height'])),
            include(CTRL_PKG, 'estimation.launch.py', {
                'use_sim_time': 'false',
                'z_des': '%.6f' % float(LaunchConfiguration('zD').perform(context)),
                'quad_mass': LaunchConfiguration('quad_mass'),
                'camera_hfov': '%.9f' % cam['hfov'],
                'camera_width': str(cam['width']),
                'camera_height': str(cam['height']),
                'camera_distortion': LaunchConfiguration('camera_distortion'),
                'aD': '%.12g' % cam['aD'],
                'marker_dict': LaunchConfiguration('marker_dict'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'sensor_qos': LaunchConfiguration('sensor_qos'),
            }),
        ]

    # Starting pos_ctrl is the handover. att_ctrl off: PX4 owns attitude and rates.
    control = include(CTRL_PKG, 'control.launch.py', {
        'attitude_controller': 'false',
        'use_sim_time': 'false',
        'zD': LaunchConfiguration('zD'),
        'quad_mass': LaunchConfiguration('quad_mass'),
    })

    # px4_takeoff_gate -> estimation, then ibvs_gate -> pos_ctrl. Same ordering as SITL.
    estimation = RegisterEventHandler(OnProcessExit(
        target_action=takeoff_gate,
        on_exit=lambda event, context: (
            _estimation(context) + [gate, control_on_gate]
            if event.returncode == 0 else
            [LogInfo(msg='px4_takeoff_gate failed - the estimators will NOT be started.')])))

    control_on_gate = RegisterEventHandler(OnProcessExit(
        target_action=gate,
        on_exit=lambda event, context: (
            [control] if event.returncode == 0 else
            [LogInfo(msg='ibvs_gate failed - pos_ctrl will NOT be started, because it would '
                         'servo on the no-lock feature vector.')])))

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        out = os.path.join(
            'bags', '%s_%s' % (LaunchConfiguration('bag_prefix').perform(context),
                               datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
        # No --use-sim-time: the wall clock is the only clock here.
        return [ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out] + bag_topics(),
            output='screen')]

    viz = include(UTILS_PKG, 'viz.launch.py', {
        'foxglove': LaunchConfiguration('foxglove'),
        'camera': LaunchConfiguration('camera'),
        'use_sim_time': 'false',
    })

    return LaunchDescription(args + [
        OpaqueFunction(function=_agent),
        state_adapter,
        OpaqueFunction(function=_bridge),
        takeoff_gate,
        estimation,
        viz,
        OpaqueFunction(function=_bag),
    ])
