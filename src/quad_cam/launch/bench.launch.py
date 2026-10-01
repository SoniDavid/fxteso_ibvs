"""The estimation + control stack on the Pi with the real camera and no PX4 - a compute benchmark.

  quad_cam/camera_node -> /quad/camera/image_raw
        |
  quad_control/estimation: image_features / td_linear / td_attitude / td_attitude_desired
        |
        +---> fixed_eso ---> pos_ctrl   (att_ctrl only if attitude_controller:=true)

For flight use quad_px4's hardware.launch.py; this one has no gates and no PX4 link.
use_sim_time is false throughout - there is no /clock here and SimRate would hang.

  ros2 launch quad_cam bench.launch.py [load:=synthetic|plate|none] [attitude_controller:=false]
                                       [camera:=module3wide_2304] [zD:=1.2]
                                       [detector_backend:=nano]

load:
  synthetic  bench_feeder fakes the plant, target and a marker lock; image_features still runs,
             its outputs moved to *_probe so the feeder drives the chain. No printed target. (default)
  plate      bench_feeder fakes the plant and target only; a printed {4,6,8,10} 7x7 plate in
             front of the camera drives image_features for real.
  none       no feeder - fixed_eso / pos_ctrl stay idle (gated on ImFeat_valid).
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CTRL_PKG = 'quad_control'
CAM_PKG = 'quad_cam'
DESC_PKG = 'quad_description'


def camera_presets():
    """quad_description's preset resolver. Loaded by path: share/<pkg>/launch is not on sys.path."""
    path = os.path.join(get_package_share_directory(DESC_PKG), 'launch', 'camera_presets.py')
    spec = importlib.util.spec_from_file_location('camera_presets', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def include(pkg, name, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=launch_arguments.items())


def generate_launch_description():
    presets = camera_presets()
    args = [
        DeclareLaunchArgument('load', default_value='synthetic'),
        DeclareLaunchArgument('attitude_controller', default_value='false'),
        DeclareLaunchArgument('camera', default_value=presets.DEFAULT_CAMERA),
        DeclareLaunchArgument('zD', default_value='1.2'),
        DeclareLaunchArgument('target_scale', default_value='0.5'),
        DeclareLaunchArgument('marker_dict', default_value='7x7'),
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        DeclareLaunchArgument('af_mode', default_value='manual'),
        DeclareLaunchArgument('lens_position', default_value='0.83'),
        # BEST_EFFORT end to end: a RELIABLE pair collapsed the camera to ~1 Hz under full-graph load.
        DeclareLaunchArgument('image_qos', default_value='sensor_data'),
        # A core each for camera_node and, optionally, image_features. Empty = unpinned.
        DeclareLaunchArgument('camera_cpu', default_value='2'),
        DeclareLaunchArgument('image_features_cpu', default_value=''),
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
        DeclareLaunchArgument('cv_num_threads', default_value='1'),
    ]

    def _stack(context, *a, **k):
        load = LaunchConfiguration('load').perform(context).lower()
        if load not in ('synthetic', 'plate', 'none'):
            raise RuntimeError('load:=%s is not synthetic, plate or none.' % load)
        synthetic = load == 'synthetic'

        zD = float(LaunchConfiguration('zD').perform(context))
        scale = float(LaunchConfiguration('target_scale').perform(context))
        margin = float(LaunchConfiguration('aruco3_margin').perform(context))
        cam = presets.resolve(LaunchConfiguration('camera').perform(context), scale, zD)
        image_qos = LaunchConfiguration('image_qos').perform(context)

        out = [
            LogInfo(msg=presets.summary(cam, zD)),
            LogInfo(msg='bench: load=%s, detector %s, marker ~%.0f px, use_sim_time false'
                        % (load, LaunchConfiguration('detector_backend').perform(context),
                           presets.marker_px(cam, scale, zD))),
            include(CAM_PKG, 'camera.launch.py', {
                'width': str(cam['width']),
                'height': str(cam['height']),
                'sensor_width': str(cam['native'][0]),
                'sensor_height': str(cam['native'][1]),
                'af_mode': LaunchConfiguration('af_mode'),
                'lens_position': LaunchConfiguration('lens_position'),
                'image_qos': image_qos,
                'cpu_affinity': LaunchConfiguration('camera_cpu'),
            }),
            include(CTRL_PKG, 'estimation.launch.py', {
                'use_sim_time': 'false',
                'z_des': '%.6f' % zD,
                'camera_hfov': '%.9f' % cam['hfov'],
                'camera_width': str(cam['width']),
                'camera_height': str(cam['height']),
                'camera_distortion': str(cam['distortion']),
                'aD': '%.12g' % cam['aD'],
                'marker_dict': LaunchConfiguration('marker_dict'),
                'camera_topic': '/quad/camera/image_raw',
                'sensor_qos': 'true' if image_qos == 'sensor_data' else 'false',
                'vision_out_suffix': '_probe' if synthetic else '',
                'image_features_cpu': LaunchConfiguration('image_features_cpu'),
                'detector_backend': LaunchConfiguration('detector_backend'),
                'nano_error_correction': LaunchConfiguration('nano_error_correction'),
                'nano_border_error_rate': LaunchConfiguration('nano_border_error_rate'),
                'nano_box_filter': LaunchConfiguration('nano_box_filter'),
                'nano_max_revisited': LaunchConfiguration('nano_max_revisited'),
                'roi_margin': LaunchConfiguration('roi_margin'),
                'use_aruco3_detection': LaunchConfiguration('use_aruco3_detection'),
                'min_marker_length_ratio': '%.6f' % presets.min_marker_ratio(
                    cam, scale, zD, margin),
                'cv_num_threads': LaunchConfiguration('cv_num_threads'),
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
                output='screen', parameters=[{'feed_vision': synthetic}]))
        return out

    return LaunchDescription(args + [
        # A 2.24 MB frame overflows Fast DDS's default 512 KiB segment; every process inherits this.
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA'),
        OpaqueFunction(function=_stack),
    ])
