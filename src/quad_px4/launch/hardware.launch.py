"""The flight stack on the real aircraft: Pi camera, estimation, gates and the PX4 link.
Nothing simulated - no Gazebo, no PX4 SITL, no sim_pilot.

  ros2 launch quad_px4 hardware.launch.py

Differs from SITL, silently if wrong: no sim time, frame_yaw_offset 0, a BEST_EFFORT camera, and
a person consents to the handover.
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
        # 0.5 = the printed 450x375 mm four-marker target. aD scales with its square.
        DeclareLaunchArgument('target_scale', default_value='0.5',
                              description='printed target size relative to the reference target; '
                                          'ignored by single_marker'),
        # four_markers | single_marker (one marker on US letter; print it with make_markers.py).
        DeclareLaunchArgument('target_layout', default_value='four_markers',
                              choices=['four_markers', 'single_marker']),
        DeclareLaunchArgument('target_marker_id', default_value='4'),

        # --- the airframe -------------------------------------------------------
        # Weighed as flown, battery included. 2.0 is the thesis vehicle's; the 3S build is ~1.3 kg.
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        # Must equal MPC_THR_HOVER on the aircraft; read it off a real hover.
        DeclareLaunchArgument('hover_thrust', default_value='0.60'),
        # Velocity in pos_ctrl's Coriolis term. off, as in SITL: td differentiates EKF2's position,
        # Vicon-aided in flight and runaway unaided, into the control law.
        DeclareLaunchArgument('velocity_source', default_value='off',
                              choices=['td', 'ekf2', 'vision', 'off']),

        # --- the camera ---------------------------------------------------------
        DeclareLaunchArgument('camera', default_value=presets.DEFAULT_CAMERA),
        # inprocess: image_features opens the camera itself; no image crosses DDS (2.2 MB frames
        # through DDS fell to ~3 fps under the full stack). topic: a driver publishes on camera_topic.
        DeclareLaunchArgument('camera_source', default_value='inprocess', choices=['inprocess', 'topic']),
        # inprocess only: a grey frame on camera_topic at this rate, for looking. 0 = none.
        DeclareLaunchArgument('debug_image_hz', default_value='1.0'),
        # topic only: start quad_cam's picamera2 driver. false = an external driver on camera_topic,
        # which must run with FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA too or image_features gets nothing.
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
        # nano | opencv | hybrid (nano while locked, opencv otherwise) | roi (opencv around the last
        # detection) | nano_roi (nano around the last detection; ~2 ms locked on the Pi vs ~22 ms full frame).
        DeclareLaunchArgument('detector_backend', default_value='nano_roi'),
        # A camera that stops sending frames reads as lock lost after this long, not as a frozen lock.
        DeclareLaunchArgument('image_timeout', default_value='0.2'),
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
        # opencv path only: none | subpix | contour.
        DeclareLaunchArgument('corner_refinement', default_value='none',
                              choices=['none', 'subpix', 'contour']),
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
        # Continuous marker lock (s) before consent hands over: the estimators' start-up transient
        # put a 4 deg roll step on a handover 0.6 s after the first lock. 0 = consent alone.
        DeclareLaunchArgument('min_lock_before_handover', default_value='3.0'),
        # Tighter than the node's 0.4 m default, which fires during the climb.
        DeclareLaunchArgument('handover_tolerance', default_value='0.15'),
        # advisory: pos_ctrl starts with the estimators and ibvs_gate only reports. block: SITL's
        # ordering. Either way pos_ctrl sends no setpoint without a marker lock.
        DeclareLaunchArgument('ibvs_gate', default_value='advisory', choices=['advisory', 'block']),
        # ibvs_gate's alignment limit, tuned against a simulated pilot's perfect hover.
        DeclareLaunchArgument('max_feature_error', default_value='0.15'),
        # The advisory report's limit: hand-held and real-world aiming is rougher than SITL's.
        DeclareLaunchArgument('advisory_feature_error', default_value='0.5'),
        DeclareLaunchArgument('gate_timeout', default_value='120.0'),
        DeclareLaunchArgument('estimators_ready', default_value='5.0'),
        # Barometer-only height. The SITL barometer is 300x quieter than PX4's own model.
        DeclareLaunchArgument('takeoff_tolerance', default_value='0.20'),

        # --- Vicon aiding: the receiver runs on the laptop (x86-64 SDK) with LARGE_DATA transport ---
        DeclareLaunchArgument('vicon', default_value='false',
                              description='feed Vicon to EKF2 as external vision'),
        DeclareLaunchArgument('vicon_topic', default_value='/vicon/fxteso_f450quad/fxteso_f450quad',
                              description='PoseStamped from the Vicon receiver'),
        # gated: aided iff the loop does not have the aircraft, so the scored window is unaided.
        # always: bring-up and shadow flights. oneshot: until the first handover. manual: service.
        DeclareLaunchArgument('aiding_policy', default_value='gated',
                              choices=['gated', 'always', 'oneshot', 'manual']),
        # The Vicon global frame from the wand calibration, and the object template's body axes.
        # Both are guesses until step 2 of the bring-up ladder confirms them.
        DeclareLaunchArgument('vicon_frame', default_value='enu', choices=['enu', 'ned']),
        DeclareLaunchArgument('body_frame', default_value='flu', choices=['flu', 'frd']),
        # Rotates the Vicon world onto the workspace datum. Measured on site, like frame_yaw_offset.
        DeclareLaunchArgument('vicon_yaw_offset', default_value='0.0'),
        # header needs chrony between the laptop and the Pi; receipt carries the latency in
        # EKF2_EV_DELAY instead.
        DeclareLaunchArgument('stamp_source', default_value='header',
                              choices=['header', 'receipt']),

        # --- the workspace datum: zero (SITL's -pi/2 is Gazebo's ENU); Vicon uses vicon_yaw_offset ---
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
            _float('target_scale', context), _float('zD', context),
            LaunchConfiguration('target_layout').perform(context),
            LaunchConfiguration('marker_dict').perform(context))
        w = LaunchConfiguration('camera_width').perform(context)
        h = LaunchConfiguration('camera_height').perform(context)
        if w:
            cam['width'] = int(w)
        if h:
            cam['height'] = int(h)
        # fx follows the width; aD is metric and does not.
        cam['fx'] = cam['width'] / (2.0 * math.tan(cam['hfov'] / 2.0))
        return cam

    def _inprocess(context):
        return LaunchConfiguration('camera_source').perform(context) == 'inprocess'

    def _camera_driver(context, *a, **k):
        # inprocess: image_features holds the camera, and a second opener would find it busy.
        if _inprocess(context) or LaunchConfiguration('camera_driver').perform(context).lower() != 'true':
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

    def _vicon(context, *a, **k):
        if LaunchConfiguration('vicon').perform(context).lower() != 'true':
            return []
        return [
            LogInfo(msg='vicon: EKF2 external-vision aiding ON, policy %s. Load params/hardware.params%s '
                        'and REBOOT the FC first.'
                        % (LaunchConfiguration('aiding_policy').perform(context),
                           ' then params/hardware_vicon.params'
                           if LaunchConfiguration('aiding_policy').perform(context) == 'always'
                           else '')),
            Node(package=PKG, executable='vicon_px4_bridge', name='vicon_px4_bridge',
                 output='screen',
                 parameters=[{'use_sim_time': False,
                              'vicon_topic': LaunchConfiguration('vicon_topic'),
                              'aiding_policy': LaunchConfiguration('aiding_policy'),
                              'vicon_frame': LaunchConfiguration('vicon_frame'),
                              'body_frame': LaunchConfiguration('body_frame'),
                              'yaw_offset': ParameterValue(
                                  LaunchConfiguration('vicon_yaw_offset'), value_type=float),
                              'stamp_source': LaunchConfiguration('stamp_source')}]),
        ]

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
                         'min_lock_before_handover': ParameterValue(
                             LaunchConfiguration('min_lock_before_handover'), value_type=float),
                         # Known harmful: it defeats the offboard-loss failsafe.
                         'offboard_recovery': False,
                         'frame_yaw_offset': ParameterValue(
                             LaunchConfiguration('frame_yaw_offset'), value_type=float)}])]

    def _estimation(context, *a, **k):
        cam = _camera(context)
        zD = _float('zD', context)
        scale = _float('target_scale', context)
        layout = LaunchConfiguration('target_layout').perform(context)
        mdict = LaunchConfiguration('marker_dict').perform(context)
        return [
            LogInfo(msg=presets.summary(cam, zD)),
            LogInfo(msg='target %s, %s%s' % (
                layout, mdict, ', target_scale ignored' if layout == 'single_marker'
                else ', scale %g' % scale)),
            LogInfo(msg='hardware: use_sim_time FALSE, frame_yaw_offset %s, camera %dx%d, '
                        'marker ~%.0f px, detector %s'
                        % (LaunchConfiguration('frame_yaw_offset').perform(context),
                           cam['width'], cam['height'],
                           presets.marker_px(cam, scale, zD, layout, mdict),
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
                'image_timeout': LaunchConfiguration('image_timeout'),
                'image_source': 'libcamera' if _inprocess(context) else 'topic',
                'camera_sensor_width': str(cam['native'][0]),
                'camera_sensor_height': str(cam['native'][1]),
                'camera_fps': str(cam['fps']),
                'af_mode': LaunchConfiguration('af_mode'),
                'lens_position': LaunchConfiguration('lens_position'),
                'debug_image_hz': LaunchConfiguration('debug_image_hz'),
                'use_aruco3_detection': LaunchConfiguration('use_aruco3_detection'),
                'min_marker_length_ratio': '%.6f' % presets.min_marker_ratio(
                    cam, scale, zD, _float('aruco3_margin', context), layout, mdict),
                'cv_num_threads': LaunchConfiguration('cv_num_threads'),
                'image_features_cpu': LaunchConfiguration('image_features_cpu'),
                'target_layout': layout,
                'target_marker_id': LaunchConfiguration('target_marker_id'),
                'corner_refinement': LaunchConfiguration('corner_refinement'),
            }),
        ]

    # Starting pos_ctrl is the handover. att_ctrl off: PX4 owns attitude and rates.
    control = include(CTRL_PKG, 'control.launch.py', {
        'attitude_controller': 'false',
        'use_sim_time': 'false',
        'zD': LaunchConfiguration('zD'),
        'quad_mass': LaunchConfiguration('quad_mass'),
        'velocity_source': LaunchConfiguration('velocity_source'),
    })

    def _gated_control(context):
        """ibvs_gate, and pos_ctrl either beside it (advisory) or on its exit (block)."""
        advisory = LaunchConfiguration('ibvs_gate').perform(context) == 'advisory'
        limit = 'advisory_feature_error' if advisory else 'max_feature_error'
        # require_target false: tgt_position is the simulator's, and the target here is a printed one.
        gate = Node(
            package=CTRL_PKG, executable='ibvs_gate', name='ibvs_gate', output='screen',
            parameters=[{'use_sim_time': False,
                         'max_feature_error': ParameterValue(
                             LaunchConfiguration(limit), value_type=float),
                         'estimators_ready': ParameterValue(
                             LaunchConfiguration('estimators_ready'), value_type=float),
                         'timeout': ParameterValue(
                             LaunchConfiguration('gate_timeout'), value_type=float),
                         'require_target': False}])
        if advisory:
            report = RegisterEventHandler(OnProcessExit(
                target_action=gate,
                on_exit=lambda event, context: [LogInfo(msg=(
                    'ibvs_gate (advisory): OK - aligned marker lock within %s.'
                    if event.returncode == 0 else
                    'ibvs_gate (advisory): no aligned lock within %s before its timeout; '
                    'pos_ctrl is running regardless.') % LaunchConfiguration(limit).perform(context))]))
            return [LogInfo(msg='ibvs_gate advisory: pos_ctrl starts now and servos on any marker '
                                'lock; OFFBOARD still needs consent and a live lock.'),
                    control, gate, report]
        on_gate = RegisterEventHandler(OnProcessExit(
            target_action=gate,
            on_exit=lambda event, context: (
                [control] if event.returncode == 0 else
                [LogInfo(msg='ibvs_gate failed - pos_ctrl will NOT be started, because it would '
                             'servo on the no-lock feature vector.')])))
        return [gate, on_gate]

    # px4_takeoff_gate -> estimation, then ibvs_gate and pos_ctrl as ibvs_gate:= says.
    estimation = RegisterEventHandler(OnProcessExit(
        target_action=takeoff_gate,
        on_exit=lambda event, context: (
            _estimation(context) + _gated_control(context)
            if event.returncode == 0 else
            [LogInfo(msg='px4_takeoff_gate failed - the estimators will NOT be started.')])))

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
        # Before the state adapter and the bridge: EKF2 should be aided by the time anything
        # downstream reads its estimate.
        OpaqueFunction(function=_vicon),
        state_adapter,
        OpaqueFunction(function=_bridge),
        takeoff_gate,
        estimation,
        viz,
        OpaqueFunction(function=_bag),
    ])
