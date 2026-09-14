"""The IBVS estimation + control stack on the real aircraft computer - no Gazebo.

  quad_cam/camera_node -> /quad/camera/image_raw
        |
  quad_control/estimation: image_features / td_linear / td_attitude / td_attitude_desired
        |
        +---> fixed_eso ---> pos_ctrl   (att_ctrl only if attitude_controller:=true)

Runs everything that ships to the aircraft and nothing simulated - no gz, no PX4 bridge,
no ibvs_gate (adding the pilot/PX4 path is a later revision). Every node runs with
use_sim_time:=false; there is no /clock here and leaving it true hangs SimRate.

Self-contained on purpose: quad_gz_sim (which owns the camera preset table) needs a full
Gazebo toolchain and does not build on the Pi, so the module3wide geometry is inlined
below - keep it in sync with quad_gz_sim/config/cameras.yaml.

  ros2 launch quad_cam hardware.launch.py [load:=synthetic|plate|none]
                                          [attitude_controller:=false]
                                          [camera:=module3wide_2304] [zD:=1.2]

load:
  synthetic  bench_feeder fakes the plant, target and a marker lock; image_features
             still runs but its feature outputs go to *_probe so the feeder drives the
             chain. No printed target needed. (default)
  plate      bench_feeder fakes the plant/target only; put a real {4,6,8,10} 7x7 ArUco
             plate in front of the camera to drive image_features for real.
  none       no feeder - fixed_eso / pos_ctrl stay idle (gated on ImFeat_valid).
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CTRL_PKG = 'quad_control'
CAM_PKG = 'quad_cam'

# quad_gz_sim/config/cameras.yaml, render size + effective hFOV + the assumed barrel
# profile. image_features derives fx from hfov+width itself, so only these are passed.
_PRESETS = {
    'module3wide_2304':       dict(hfov_deg=102.0, w=1152, h=648,
                                   dist=[-0.30, 0.10, 0.0, 0.0, -0.02]),
    'module3wide_2304_ideal': dict(hfov_deg=102.0, w=1152, h=648,
                                   dist=[0.0, 0.0, 0.0, 0.0, 0.0]),
    'module3_2304':           dict(hfov_deg=66.0, w=1152, h=648,
                                   dist=[0.0, 0.0, 0.0, 0.0, 0.0]),
}
# camera_presets.py: aD = CENTROID_MOMENT * target_scale**2 * (FOCAL_LENGTH / zD)**2
_FOCAL_LENGTH = 0.00304
_CENTROID_MOMENT = 0.5625
# printable-target.md: marker ink (black edge to black edge) is 244.5 mm at target_scale=1.0
_MARKER_INK_M = 0.2445


def include(pkg, name, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=launch_arguments.items())


def generate_launch_description():
    args = [
        DeclareLaunchArgument('load', default_value='synthetic'),
        DeclareLaunchArgument('attitude_controller', default_value='false'),
        DeclareLaunchArgument('camera', default_value='module3wide_2304'),
        DeclareLaunchArgument('zD', default_value='1.2'),
        DeclareLaunchArgument('target_scale', default_value='0.5'),
        DeclareLaunchArgument('marker_dict', default_value='7x7'),
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        DeclareLaunchArgument('af_mode', default_value='manual'),
        DeclareLaunchArgument('lens_position', default_value='0.83'),
        # BEST_EFFORT end-to-end by default: a RELIABLE camera_node paired with a
        # struggling RELIABLE image_features subscriber can back-pressure the publisher
        # under full-graph CPU load (observed: /quad/camera/image_raw collapsing to ~1 Hz,
        # bursty). sensor_data drops frames (latest-wins) instead of stalling the writer.
        DeclareLaunchArgument('image_qos', default_value='sensor_data'),
        # CPU pinning: on a 4-core Pi, the full graph (8 processes) can starve
        # camera_node's and image_features' own threads of scheduling even with plenty of
        # kernel socket buffer - the publisher reports a healthy fps while the subscriber
        # receives far fewer frames than were sent. Dedicating them a core each removes
        # that contention. Empty = unpinned; '' here is the safe default for camera.launch.py
        # and estimation.launch.py used standalone, but hardware.launch.py defaults to
        # pinning since that's where the contention was observed.
        DeclareLaunchArgument('camera_cpu', default_value='2'),
        DeclareLaunchArgument('image_features_cpu', default_value=''),
        # Speeds up cv::aruco::detectMarkers without touching resolution/FOV: the
        # candidate-quad search runs on a downscaled image instead of the full frame,
        # while corner refinement/decoding still happens at full resolution (see
        # image_features.cpp). 'true' by default here - this is the hardware-only launch
        # file the feature was built for; estimation.launch.py itself defaults it off so
        # a bare/SITL launch is unaffected.
        DeclareLaunchArgument('use_aruco3_detection', default_value='true'),
        # The fraction of the nominal marker size (computed below, from zD/target_scale)
        # that must still resolve in the downscaled search. 0.7 protects markers 30% below
        # the deployed operating size - margin for oblique viewing angle and trajectory
        # overshoot - without giving up most of the speedup by protecting all the way down
        # to DICT_7X7_50's ~30 px hard decode floor (camera-geometry.md).
        DeclareLaunchArgument('aruco3_margin', default_value='0.7'),
        # A/B knob, see image_features.cpp - 0 leaves TBB's own thread count alone.
        DeclareLaunchArgument('cv_num_threads', default_value='0'),
        # 'opencv' (default) or 'nano' - see image_features.cpp.
        DeclareLaunchArgument('detector_backend', default_value='opencv'),
    ]

    camera = include(CAM_PKG, 'camera.launch.py', {
        'af_mode': LaunchConfiguration('af_mode'),
        'lens_position': LaunchConfiguration('lens_position'),
        'image_qos': LaunchConfiguration('image_qos'),
        'cpu_affinity': LaunchConfiguration('camera_cpu'),
    })

    def _stack(context, *a, **k):
        load = LaunchConfiguration('load').perform(context).lower()
        if load not in ('synthetic', 'plate', 'none'):
            raise RuntimeError("load:=%s is not synthetic, plate or none." % load)
        synthetic = load == 'synthetic'

        name = LaunchConfiguration('camera').perform(context)
        if name not in _PRESETS:
            raise RuntimeError('camera:=%s is not one of %s'
                               % (name, ', '.join(_PRESETS)))
        p = _PRESETS[name]
        zD = float(LaunchConfiguration('zD').perform(context))
        scale = float(LaunchConfiguration('target_scale').perform(context))
        hfov = math.radians(p['hfov_deg'])
        aD = _CENTROID_MOMENT * scale ** 2 * (_FOCAL_LENGTH / zD) ** 2

        # Same fx image_features derives internally (hfov, width) -> nominal marker size
        # in the published image at this zD/target_scale -> the useAruco3Detection safety
        # floor, as a fraction of margin below nominal. See image_features.cpp.
        fx = p['w'] / (2.0 * math.tan(hfov / 2.0))
        marker_px_nominal = _MARKER_INK_M * scale * fx / zD
        margin = float(LaunchConfiguration('aruco3_margin').perform(context))
        min_marker_ratio = (margin * marker_px_nominal) / max(p['w'], p['h'])

        out = [
            LogInfo(msg='hardware.launch.py: camera=%s %dx%d hFOV %.1f deg, aD %.4e, '
                        'zD %.2f m | marker ~%.0f px, aruco3 protects >=%.0f px '
                        '(ratio %.4f) | load=%s | use_sim_time=false'
                        % (name, p['w'], p['h'], p['hfov_deg'], aD, zD, marker_px_nominal,
                           margin * marker_px_nominal, min_marker_ratio, load)),
            include(CTRL_PKG, 'estimation.launch.py', {
                'use_sim_time': 'false',
                'z_des': LaunchConfiguration('zD'),
                'camera_hfov': '%.9f' % hfov,
                'camera_width': str(p['w']),
                'camera_height': str(p['h']),
                'camera_distortion': str(p['dist']),
                'aD': '%.12g' % aD,
                'marker_dict': LaunchConfiguration('marker_dict'),
                'camera_topic': '/quad/camera/image_raw',
                'vision_out_suffix': '_probe' if synthetic else '',
                'image_qos': LaunchConfiguration('image_qos'),
                'image_features_cpu': LaunchConfiguration('image_features_cpu'),
                'use_aruco3_detection': LaunchConfiguration('use_aruco3_detection'),
                'min_marker_length_ratio': '%.6f' % min_marker_ratio,
                'cv_num_threads': LaunchConfiguration('cv_num_threads'),
                'detector_backend': LaunchConfiguration('detector_backend'),
            }),
            include(CTRL_PKG, 'control.launch.py', {
                'use_sim_time': 'false',
                'zD': LaunchConfiguration('zD'),
                'quad_mass': LaunchConfiguration('quad_mass'),
                'attitude_controller': LaunchConfiguration('attitude_controller'),
            }),
        ]
        if load != 'none':
            out.append(Node(
                package=CAM_PKG, executable='bench_feeder', name='bench_feeder',
                output='screen', parameters=[{'feed_vision': synthetic}],
                additional_env={'FASTDDS_BUILTIN_TRANSPORTS': 'LARGE_DATA'}))
        return out

    return LaunchDescription(args + [camera, OpaqueFunction(function=_stack)])
