"""The estimation chain: camera and plant state in, feature and state estimates out.

Safe to run without the controllers - that is the open-loop observer test. Each node
staggers its own start on sim time, so nothing here needs a launch delay.

The camera arguments are plain numbers on purpose. quad_control carries no simulation
dependency, so it must not read quad_gz_sim's camera preset table; the caller resolves the
preset once and passes the same values here and to gz_sim.launch.py. Their defaults are the
Camera Module 2 geometry image_features had hardcoded, so an unadorned launch is unchanged.
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
        # The observer's desired servoing depth, thesis Eq. 5.81's z_d, which sets g_xi = -1/z_d
        # on x/y/z. THE SAME QUANTITY AS pos_ctrl's zD - every composer below passes its own zD
        # here. The 2.5 default is only for running this file on its own; it matches the zD
        # default of sim.launch.py and observer_only.launch.py.
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
        # The image moment at the desired depth: aD = 0.5625 * target_scale^2 * (f/zD)^2, with
        # f = 0.00304. The default is the flown constant, which that formula reproduces at
        # 2.605 m - so aD, not zD, is what sets where the aircraft actually settles.
        DeclareLaunchArgument('aD', default_value='0.00000076589'),
        DeclareLaunchArgument('marker_dict', default_value='7x7',
                              description='must match the textures gz_sim.launch.py selected'),
        # The camera topic image_features reads. A remapping, so a distortion-applying node can
        # be inserted upstream without image_features knowing.
        DeclareLaunchArgument('camera_topic', default_value='/quad/camera/image_raw'),
        # Airframe mass as flown. fixed_eso and pos_ctrl must get the same number.
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
        # true in SITL (nodes run on the bridged /clock); false on the aircraft, where
        # there is no /clock and SimRate would otherwise hang on node->now(). The default
        # keeps sim.launch.py / sitl.launch.py unchanged - they do not forward this.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Benchmark only: when set (e.g. '_probe'), image_features' feature outputs are
        # remapped aside so a synthetic feeder can drive the downstream chain while
        # image_features still runs for its real cost. Empty = normal wiring.
        DeclareLaunchArgument('vision_out_suffix', default_value=''),
        # 'reliable' (default, matches ros_gz_bridge in SITL) or 'sensor_data' - chosen
        # together with the publisher, since a RELIABLE subscription against a BEST_EFFORT
        # publisher receives nothing, and the reverse pairing can back-pressure a
        # struggling publisher.
        DeclareLaunchArgument('image_qos', default_value='reliable'),
        # Pin image_features to a CPU core, e.g. '3' - empty (default) leaves it unpinned.
        # image_features is the heaviest node in the graph (cv::aruco::detectMarkers), and
        # under load its own SimRate loop can fall well behind (see fixed_eso's neighbours
        # for the achieved-rate picture); pinning removes competition for its core.
        DeclareLaunchArgument('image_features_cpu', default_value=''),
        # False (OpenCV's own default) keeps a bare launch - what SITL uses - unchanged.
        # hardware.launch.py turns this on with min_marker_length_ratio computed from the
        # deployed zD/target_scale; see image_features.cpp for what the pair does.
        DeclareLaunchArgument('use_aruco3_detection', default_value='false'),
        DeclareLaunchArgument('min_marker_length_ratio', default_value='0.0'),
        # 0 (default) leaves OpenCV's TBB thread count untouched. Pair with
        # image_features_cpu for an A/B - see image_features.cpp.
        DeclareLaunchArgument('cv_num_threads', default_value='0'),
        # 'opencv' (default) keeps the validated behavior; 'nano' switches to aruco_nano -
        # a different detection algorithm, opt-in until field-validated. See image_features.cpp.
        DeclareLaunchArgument('detector_backend', default_value='opencv'),
    ]

    def params(exe):
        # Each node declares only its own parameters; handing one the others' would fail its
        # launch, so these are split by executable rather than passed to everything.
        p = {'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool)}
        if exe == 'image_features':
            p['image_qos'] = ParameterValue(
                LaunchConfiguration('image_qos'), value_type=str)
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
            p['use_aruco3_detection'] = ParameterValue(
                LaunchConfiguration('use_aruco3_detection'), value_type=bool)
            p['min_marker_length_ratio'] = ParameterValue(
                LaunchConfiguration('min_marker_length_ratio'), value_type=float)
            p['cv_num_threads'] = ParameterValue(
                LaunchConfiguration('cv_num_threads'), value_type=int)
            p['detector_backend'] = ParameterValue(
                LaunchConfiguration('detector_backend'), value_type=str)
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
        if exe != 'image_features':
            return []
        suffix = LaunchConfiguration('vision_out_suffix')
        # Identity when vision_out_suffix is empty (the SITL default).
        return [('/quad/camera/image_raw', LaunchConfiguration('camera_topic'))] + [
            (t, [t, suffix]) for t in ('ImFeat_vector', 'ImFeat_valid', 'a_value')]

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
                 # Fast DDS's default shared-memory segment is 512 KiB; image_features'
                 # incoming frame (1152x648 bgr8) is 2.24 MB. Under CPU contention a
                 # publisher this much larger than the segment gets silently dropped well
                 # before the OS network layer, even though the publisher reports a
                 # healthy fps - the LARGE_DATA transport profile raises the segment (and
                 # the UDP fallback's max message size) to fit. Every quad_cam / quad_control
                 # participant sets this the same way - see camera.launch.py.
                 additional_env={'FASTDDS_BUILTIN_TRANSPORTS': 'LARGE_DATA'},
                 **prefix(exe, context))
            for exe, verbose in NODES
        ]

    return LaunchDescription(args + [OpaqueFunction(function=nodes)])
