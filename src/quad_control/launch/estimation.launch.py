"""The estimation chain: camera and plant state in, feature and state estimates out. Runs without
the controllers too. Camera arguments are plain numbers: the caller resolves the preset.
"""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'quad_control'

NODES = [
    ('image_features',       True),   # prints "I see N arucos only" when it loses lock
    ('td_attitude',          False),
    ('td_attitude_desired',  False),
    ('td_linear',            False),
    ('fixed_eso',            False),
]


def generate_launch_description():
    args = [DeclareLaunchArgument(
        'gamma1_xy', default_value='18.0',
        description="fixed_eso's x/y position-injection gain, the thesis' 18 on every plant. "
                    'px4 flew 9 until its rotor model was corrected; the two are now '
                    'indistinguishable there.'),
        DeclareLaunchArgument(
            'observer_omega', default_value='0.0',
            description="Above zero, replaces fixed_eso's x/y gains with a triple pole at "
                        '-observer_omega rad/s, overriding gamma1_xy. Zero keeps the flown set.'),
        DeclareLaunchArgument('gamma2_xy', default_value='20.0'),
        DeclareLaunchArgument('gamma3_xy', default_value='4.0'),
        DeclareLaunchArgument('gamma3_yaw', default_value='3.0'),
        DeclareLaunchArgument('gamma1_yaw', default_value='5.0'),
        DeclareLaunchArgument('gamma2_yaw', default_value='16.0'),
        DeclareLaunchArgument('alpha_yaw', default_value='0.75'),
        DeclareLaunchArgument('beta_yaw', default_value='1.2'),
        DeclareLaunchArgument('gamma4_yaw', default_value='0.001'),
        # The observer's g(xi)*u sign on yaw. 1.0 is the as-flown value; -1.0 is
        # thesis Eq. 5.81's g_xi,psi = -1. See fixed_eso.cpp.
        DeclareLaunchArgument('eso_yaw_sign', default_value='1.0'),
        # Thesis Eq. 5.81's z_d (g_xi = -1/z_d): the same quantity as pos_ctrl's zD.
        DeclareLaunchArgument('z_des', default_value='2.5'),
        DeclareLaunchArgument(
            'initial_estimate_offset', default_value='[0.0, 0.0, 0.0, 0.0]',
            description="Added to fixed_eso's initial state estimate, i.e. the seeded initial "
                        'estimation error (qx,qy,qz,qpsi). Zeros = the thesis start.'),
        # Intrinsics of the camera actually rendered. Only pixel_size/fx enters the feature
        # model, so fx is derived from these rather than carried separately.
        DeclareLaunchArgument('camera_hfov', default_value='1.085595'),
        DeclareLaunchArgument('camera_width', default_value='820'),
        DeclareLaunchArgument('camera_height', default_value='616'),
        # Brown-Conrady k1,k2,p1,p2,k3. All zero keeps undistortPoints out of the path; it only
        # ever touches the four marker centroids, so it costs microseconds, not a frame.
        DeclareLaunchArgument('camera_distortion', default_value='[0.0, 0.0, 0.0, 0.0, 0.0]'),
        # Image moment at the desired depth, aD = moment * (f/zD)^2 (camera_presets.target_geometry).
        DeclareLaunchArgument('aD', default_value='0.00000076589'),
        # four_markers: centroids of markers 4/6/8/10. single_marker: one marker's corners, yaw
        # from its edges.
        DeclareLaunchArgument('target_layout', default_value='four_markers'),
        DeclareLaunchArgument('target_marker_id', default_value='4'),
        # opencv path only (nano always refines): none | subpix | contour.
        DeclareLaunchArgument('corner_refinement', default_value='none'),
        DeclareLaunchArgument('marker_dict', default_value='7x7',
                              description='must match the textures gz_sim.launch.py selected'),
        # The camera topic image_features reads, so a distortion node or a real driver can sit
        # upstream without touching the source.
        DeclareLaunchArgument('camera_topic', default_value='/quad/camera/image_raw'),
        # Its CameraInfo, which image_features prefers over the parameters above whenever one
        # arrives - how a calibrated real lens reaches the feature model without a rebuild.
        DeclareLaunchArgument('camera_info_topic', default_value='/quad/camera/camera_info'),
        # True for a real driver: they publish BEST_EFFORT and a RELIABLE subscription to one
        # receives nothing. ros_gz_bridge is RELIABLE, so simulation cannot surface it.
        DeclareLaunchArgument('sensor_qos', default_value='false'),
        # False on hardware: SimRate blocks on node->now(), so with no /clock this hangs.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Airframe mass as flown. fixed_eso and pos_ctrl must get the same number.
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        # Benchmark only: e.g. '_probe' moves the feature outputs aside so a feeder drives the chain.
        DeclareLaunchArgument('vision_out_suffix', default_value=''),
        # Pin image_features, the heaviest node, to a core, e.g. '3'. Empty leaves it unpinned.
        DeclareLaunchArgument('image_features_cpu', default_value=''),
        # The attitude image_features de-rotates by. SITL diagnostics point it at a ground-truth
        # reference (sitl.launch.py attitude_oracle); everything else leaves it on the estimate.
        DeclareLaunchArgument('attitude_topic', default_value='attitude_estimates'),
        # nano | opencv | hybrid (nano while locked, opencv otherwise) | roi (opencv around the last detection).
        DeclareLaunchArgument('detector_backend', default_value='hybrid'),
        # opencv arm only: candidate search on a downscaled image. The ratio is inert without it.
        DeclareLaunchArgument('use_aruco3_detection', default_value='false'),
        DeclareLaunchArgument('min_marker_length_ratio', default_value='0.0'),
        # 0 leaves OpenCV's thread count alone; at 1 its idle pool stops burning CPU next to nano.
        DeclareLaunchArgument('cv_num_threads', default_value='1'),
        # aruco_nano tolerances; nano's own 0 / 0 rejects a marker for one mis-read bit.
        DeclareLaunchArgument('nano_error_correction', default_value='0.3'),
        DeclareLaunchArgument('nano_border_error_rate', default_value='0.35'),
        DeclareLaunchArgument('nano_box_filter', default_value='15'),
        DeclareLaunchArgument('nano_max_revisited', default_value='0.05'),
        # roi backend: margin on each side of the last target box, as a fraction of its size.
        DeclareLaunchArgument('roi_margin', default_value='0.5'),
        # s without a new frame before the lock is reported lost; 0 keeps the last frame forever.
        DeclareLaunchArgument('image_timeout', default_value='0.0'),
        # topic: frames over DDS from camera_topic. libcamera: image_features opens the Pi camera.
        DeclareLaunchArgument('image_source', default_value='topic', choices=['topic', 'libcamera']),
        # libcamera only: sensor mode, rate, orientation, focus, and a low-rate grey debug image.
        DeclareLaunchArgument('camera_sensor_width', default_value='2304'),
        DeclareLaunchArgument('camera_sensor_height', default_value='1296'),
        DeclareLaunchArgument('camera_fps', default_value='56.0'),
        DeclareLaunchArgument('camera_rotate180', default_value='true'),
        DeclareLaunchArgument('af_mode', default_value='manual'),
        DeclareLaunchArgument('lens_position', default_value='0.83'),
        DeclareLaunchArgument('debug_image_hz', default_value='0.0'),
    ]

    def params(exe):
        # Each node declares only its own parameters; handing one the others' would fail its
        # launch, so these are split by executable rather than passed to everything.
        p = {'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool)}
        if exe == 'image_features':
            p['camera_hfov'] = ParameterValue(
                LaunchConfiguration('camera_hfov'), value_type=float)
            p['camera_width'] = ParameterValue(
                LaunchConfiguration('camera_width'), value_type=int)
            p['camera_height'] = ParameterValue(
                LaunchConfiguration('camera_height'), value_type=int)
            p['camera_distortion'] = ParameterValue(
                LaunchConfiguration('camera_distortion'), value_type=List[float])
            p['aD'] = ParameterValue(LaunchConfiguration('aD'), value_type=float)
            p['marker_dict'] = ParameterValue(
                LaunchConfiguration('marker_dict'), value_type=str)
            # Parameters, not remaps: the QoS is chosen for the same publisher.
            p['image_topic'] = ParameterValue(
                LaunchConfiguration('camera_topic'), value_type=str)
            p['camera_info_topic'] = ParameterValue(
                LaunchConfiguration('camera_info_topic'), value_type=str)
            p['sensor_qos'] = ParameterValue(
                LaunchConfiguration('sensor_qos'), value_type=bool)
            p['detector_backend'] = ParameterValue(
                LaunchConfiguration('detector_backend'), value_type=str)
            p['use_aruco3_detection'] = ParameterValue(
                LaunchConfiguration('use_aruco3_detection'), value_type=bool)
            p['min_marker_length_ratio'] = ParameterValue(
                LaunchConfiguration('min_marker_length_ratio'), value_type=float)
            p['cv_num_threads'] = ParameterValue(
                LaunchConfiguration('cv_num_threads'), value_type=int)
            for name, kind in (('nano_error_correction', float), ('nano_border_error_rate', float),
                               ('nano_box_filter', int), ('nano_max_revisited', float),
                               ('roi_margin', float), ('image_timeout', float), ('target_layout', str),
                               ('image_source', str), ('camera_sensor_width', int),
                               ('camera_sensor_height', int), ('camera_fps', float),
                               ('camera_rotate180', bool), ('af_mode', str),
                               ('lens_position', float), ('debug_image_hz', float),
                               ('target_marker_id', int), ('corner_refinement', str)):
                p[name] = ParameterValue(LaunchConfiguration(name), value_type=kind)
        if exe == 'fixed_eso':
            p['gamma1_xy'] = ParameterValue(LaunchConfiguration('gamma1_xy'), value_type=float)
            p['observer_omega'] = ParameterValue(
                LaunchConfiguration('observer_omega'), value_type=float)
            for g in ('gamma2_xy', 'gamma3_xy', 'gamma3_yaw', 'gamma1_yaw', 'gamma2_yaw',
                      'alpha_yaw', 'beta_yaw', 'gamma4_yaw', 'eso_yaw_sign'):
                p[g] = ParameterValue(LaunchConfiguration(g), value_type=float)
            p['z_des'] = ParameterValue(LaunchConfiguration('z_des'), value_type=float)
            p['quad_mass'] = ParameterValue(LaunchConfiguration('quad_mass'), value_type=float)
            p['initial_estimate_offset'] = ParameterValue(
                LaunchConfiguration('initial_estimate_offset'), value_type=List[float])
        return [p]

    def remaps(exe):
        # Outputs are the identity while vision_out_suffix is empty; so is the attitude input
        # while attitude_topic is left at its default.
        if exe != 'image_features':
            return []
        suffix = LaunchConfiguration('vision_out_suffix')
        return ([(t, [t, suffix]) for t in ('ImFeat_vector', 'ImFeat_valid', 'a_value')]
                + [('attitude_estimates', LaunchConfiguration('attitude_topic'))])

    def prefix(exe, context):
        if exe != 'image_features':
            return {}
        cpu = LaunchConfiguration('image_features_cpu').perform(context)
        return {'prefix': 'taskset -c ' + cpu} if cpu else {}

    def nodes(context, *a, **k):
        return [
            Node(package=PKG, executable=exe, name=exe,
                 output='screen' if verbose else 'log',
                 remappings=remaps(exe),
                 parameters=params(exe),
                 **prefix(exe, context))
            for exe, verbose in NODES
        ]

    return LaunchDescription(args + [OpaqueFunction(function=nodes)])
