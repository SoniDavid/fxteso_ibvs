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
from launch.actions import DeclareLaunchArgument
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
    ]

    def params(exe):
        # Each node declares only its own parameters; handing one the others' would fail its
        # launch, so these are split by executable rather than passed to everything.
        p = {'use_sim_time': True}
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
        if exe == 'fixed_eso':
            p['gamma1_xy'] = ParameterValue(LaunchConfiguration('gamma1_xy'), value_type=float)
            p['observer_omega'] = ParameterValue(
                LaunchConfiguration('observer_omega'), value_type=float)
            for g in ('gamma2_xy', 'gamma3_xy', 'gamma3_yaw', 'gamma1_yaw', 'gamma2_yaw',
                      'alpha_yaw', 'beta_yaw', 'gamma4_yaw', 'eso_yaw_sign'):
                p[g] = ParameterValue(LaunchConfiguration(g), value_type=float)
            p['initial_estimate_offset'] = ParameterValue(
                LaunchConfiguration('initial_estimate_offset'), value_type=List[float])
        return [p]

    # Every node runs on the simulator's clock; /clock is bridged in quad_gz_sim.
    def remaps(exe):
        if exe != 'image_features':
            return []
        return [('/quad/camera/image_raw', LaunchConfiguration('camera_topic'))]

    return LaunchDescription(args + [
        Node(package=PKG, executable=exe, name=exe,
             output='screen' if verbose else 'log',
             remappings=remaps(exe),
             parameters=params(exe))
        for exe, verbose in NODES
    ])
