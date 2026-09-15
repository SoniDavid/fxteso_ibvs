"""The flight stack on the real aircraft: Pi camera, estimation, gates and the PX4 link.
Nothing simulated - no Gazebo, no PX4 SITL, no sim_pilot.

  ros2 launch quad_px4 hardware.launch.py

Differs from SITL in ways that are not tuning, and every one fails silently: use_sim_time is
false, frame_yaw_offset is zero, the camera is subscribed BEST_EFFORT, and the handover needs a
person. att_ctrl stays off; PX4 owns the inner loop. camera_driver:=false leaves the camera to an
external driver already publishing on camera_topic.
"""
import datetime
import importlib.util
import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'quad_px4'
CTRL_PKG = 'quad_control'
UTILS_PKG = 'quad_utils'
CAM_PKG = 'quad_cam'
DESC_PKG = 'quad_description'   # the camera preset table; builds on the Pi, unlike quad_gz_sim

# The lab's Vicon coverage ceiling, not a tuning knob.
DEFAULT_ZD = 1.2
# Handover altitude. Above zD so the aircraft is settled, not still climbing.
DEFAULT_TAKEOFF_ALT = 1.5


def camera_presets():
    """quad_description's preset resolver. Loaded by path: share/<pkg>/launch is not on sys.path."""
    path = os.path.join(get_package_share_directory(DESC_PKG), 'launch', 'camera_presets.py')
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
    presets = camera_presets()
    args = [
        # --- the flight geometry ------------------------------------------------
        DeclareLaunchArgument('zD', default_value=str(DEFAULT_ZD),
                              description='servoing depth, m. The lab rig sets this.'),
        DeclareLaunchArgument('takeoff_alt', default_value=str(DEFAULT_TAKEOFF_ALT),
                              description='altitude the pilot flies to before the handover, m'),
        # 0.5 = the printed 450x375 mm plate. aD scales with its square.
        DeclareLaunchArgument('target_scale', default_value='0.5',
                              description='printed plate size relative to the reference target'),

        # --- the airframe -------------------------------------------------------
        # Weighed as flown, battery included. The 2.0 default is the simulated F450's.
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        # Must equal MPC_THR_HOVER on the aircraft; read it off a real hover.
        DeclareLaunchArgument('hover_thrust', default_value='0.60'),

        # --- the camera ---------------------------------------------------------
        DeclareLaunchArgument('camera', default_value=presets.DEFAULT_CAMERA),
        # Start quad_cam's picamera2 driver. false = an external driver on camera_topic, which must
        # run with FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA too or image_features receives nothing.
        DeclareLaunchArgument('camera_driver', default_value='true'),
        # What the driver publishes. Empty = the preset's render size, the one benchmarked at 50 Hz.
        DeclareLaunchArgument('camera_width', default_value=''),
        DeclareLaunchArgument('camera_height', default_value=''),
        DeclareLaunchArgument('camera_topic', default_value='/quad/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/quad/camera/camera_info'),
        # BEST_EFFORT on both ends: a RELIABLE subscriber back-pressures the driver under load.
        DeclareLaunchArgument('sensor_qos', default_value='true'),
        # Brown-Conrady k1,k2,p1,p2,k3 from a real calibration. The preset's are an assumption.
        DeclareLaunchArgument('camera_distortion', default_value='[0.0, 0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('marker_dict', default_value='7x7'),
        # Manual focus for flight; 0.83 dioptres is ~1.2 m.
        DeclareLaunchArgument('af_mode', default_value='manual'),
        DeclareLaunchArgument('lens_position', default_value='0.83'),

        # --- vision compute -----------------------------------------------------
        # nano | opencv | hybrid (nano while locked, opencv otherwise) | roi (opencv around the last detection).
        DeclareLaunchArgument('detector_backend', default_value='hybrid'),
        # aruco_nano tolerances (nano's own 0 / 0 rejects a marker for one mis-read bit).
        # roi_margin: fraction of the target box.
        DeclareLaunchArgument('nano_error_correction', default_value='0.3'),
        DeclareLaunchArgument('nano_border_error_rate', default_value='0.35'),
        DeclareLaunchArgument('nano_box_filter', default_value='15'),
        DeclareLaunchArgument('nano_max_revisited', default_value='0.05'),
        DeclareLaunchArgument('roi_margin', default_value='0.5'),
        # opencv arm only: aruco3 must still find markers at this fraction of the nominal size.
        DeclareLaunchArgument('use_aruco3_detection', default_value='false'),
        DeclareLaunchArgument('aruco3_margin', default_value='0.7'),
        # 1 stops OpenCV's idle pool burning CPU; fallback frames get slower, so bench 1 vs 2 on the Pi.
        DeclareLaunchArgument('cv_num_threads', default_value='1'),
        # Core pinning on the 4-core Pi. Empty leaves the node unpinned.
        DeclareLaunchArgument('camera_cpu', default_value='2'),
        DeclareLaunchArgument('image_features_cpu', default_value=''),

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

    def _float(name, context):
        return float(LaunchConfiguration(name).perform(context))

    def _camera(context):
        """Preset intrinsics at the resolution the driver actually publishes."""
        cam = presets.resolve(
            LaunchConfiguration('camera').perform(context),
            _float('target_scale', context), _float('zD', context))
        w = LaunchConfiguration('camera_width').perform(context)
        h = LaunchConfiguration('camera_height').perform(context)
        if w:
            cam['width'] = int(w)
        if h:
            cam['height'] = int(h)
        # fx follows the width; aD is metric and does not.
        cam['fx'] = cam['width'] / (2.0 * math.tan(cam['hfov'] / 2.0))
        return cam

    def _camera_driver(context, *a, **k):
        if LaunchConfiguration('camera_driver').perform(context).lower() != 'true':
            return []
        cam = _camera(context)
        best_effort = LaunchConfiguration('sensor_qos').perform(context).lower() == 'true'
        # Same size and QoS image_features is given below, so the two cannot disagree.
        return [include(CAM_PKG, 'camera.launch.py', {
            'width': str(cam['width']),
            'height': str(cam['height']),
            'sensor_width': str(cam['native'][0]),
            'sensor_height': str(cam['native'][1]),
            'image_topic': LaunchConfiguration('camera_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'image_qos': 'sensor_data' if best_effort else 'reliable',
            'af_mode': LaunchConfiguration('af_mode'),
            'lens_position': LaunchConfiguration('lens_position'),
            'cpu_affinity': LaunchConfiguration('camera_cpu'),
        })]

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
        zD = _float('zD', context)
        scale = _float('target_scale', context)
        return [
            LogInfo(msg=presets.summary(cam, zD)),
            LogInfo(msg='hardware: use_sim_time FALSE, frame_yaw_offset %s, camera %dx%d, '
                        'marker ~%.0f px, detector %s'
                        % (LaunchConfiguration('frame_yaw_offset').perform(context),
                           cam['width'], cam['height'], presets.marker_px(cam, scale, zD),
                           LaunchConfiguration('detector_backend').perform(context))),
            include(CTRL_PKG, 'estimation.launch.py', {
                'use_sim_time': 'false',
                'z_des': '%.6f' % zD,
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
                'detector_backend': LaunchConfiguration('detector_backend'),
                'nano_error_correction': LaunchConfiguration('nano_error_correction'),
                'nano_border_error_rate': LaunchConfiguration('nano_border_error_rate'),
                'nano_box_filter': LaunchConfiguration('nano_box_filter'),
                'nano_max_revisited': LaunchConfiguration('nano_max_revisited'),
                'roi_margin': LaunchConfiguration('roi_margin'),
                'use_aruco3_detection': LaunchConfiguration('use_aruco3_detection'),
                'min_marker_length_ratio': '%.6f' % presets.min_marker_ratio(
                    cam, scale, zD, _float('aruco3_margin', context)),
                'cv_num_threads': LaunchConfiguration('cv_num_threads'),
                'image_features_cpu': LaunchConfiguration('image_features_cpu'),
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
        # A 2.24 MB frame overflows Fast DDS's default 512 KiB shared-memory segment and is
        # dropped silently under load; every process launched below inherits the larger profile.
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA'),
        OpaqueFunction(function=_camera_driver),
        OpaqueFunction(function=_agent),
        state_adapter,
        OpaqueFunction(function=_bridge),
        takeoff_gate,
        estimation,
        viz,
        OpaqueFunction(function=_bag),
    ])
