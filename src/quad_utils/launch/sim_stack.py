"""The analytic and gazebo stacks, built once for every quad_utils entry point.

Not a launch file: analytic.launch.py, gazebo.launch.py, sim.launch.py and
observer_only.launch.py load it by path and call build(). The argument set is sitl.launch.py's
minus what only PX4 has (venue, EKF2, rotor model, takeoff, OFFBOARD, attitude oracle), so the
three plants take the same arguments wherever the argument means the same thing.
"""
import importlib.util
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'quad_utils'
SIM_PKG = 'quad_gz_sim'
CTRL_PKG = 'quad_control'

PLANTS = ('analytic', 'gazebo')

# The thesis conditions every archived analytic/gazebo bag was flown in.
THESIS = {'zD': '2.5', 'target_scale': '1.0', 'target_profile': 'thesis',
          'quad_mass': '2.0', 'start_altitude': '4.0', 'hold_target': 'false'}
# sitl.launch.py's defaults: the 3S build over the printed target at the lab depth.
DEPLOYMENT = {'zD': '1.2', 'target_scale': '0.5', 'target_profile': 'hover',
              'quad_mass': '1.30', 'start_altitude': '1.5', 'hold_target': 'true'}

# Forwarded verbatim to scenario.launch.py.
SCENARIO_ARGS = ('disturbance', 'disturbance_seed', 'gust_scale', 'wind_scale', 'gust_tau',
                 'turbulence_scale', 'wind_velocity', 'target_profile', 'target_speed',
                 'target_yaw_rate', 'target_accel', 'target_heading', 'hold_target',
                 'target_release', 'blackout_at', 'blackout_for', 'zD')
# Forwarded verbatim to estimation.launch.py.
ESTIMATION_ARGS = ('gamma1_xy', 'gamma2_xy', 'gamma3_xy', 'gamma3_yaw', 'gamma1_yaw',
                   'gamma2_yaw', 'alpha_yaw', 'beta_yaw', 'gamma4_yaw', 'eso_yaw_sign',
                   'observer_omega', 'initial_estimate_offset', 'quad_mass', 'marker_dict',
                   'detector_backend', 'nano_error_correction', 'nano_border_error_rate',
                   'nano_box_filter', 'nano_max_revisited', 'roi_margin',
                   'use_aruco3_detection', 'cv_num_threads')


def camera_presets():
    """quad_description's camera preset resolver. share/<pkg>/launch is not on sys.path, so it
    is loaded by path."""
    path = os.path.join(get_package_share_directory('quad_description'), 'launch',
                        'camera_presets.py')
    spec = importlib.util.spec_from_file_location('camera_presets', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def include(pkg, name, launch_arguments=None):
    """Include a sibling package's layer launch file."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=(launch_arguments or {}).items())


def bag_topics():
    """Flatten config/bag_topics.yaml, which groups topics by where they come from."""
    path = os.path.join(get_package_share_directory(PKG), 'config', 'bag_topics.yaml')
    with open(path) as fh:
        groups = yaml.safe_load(fh)
    return [topic for group in groups.values() for topic in group]


def _lc(names):
    return {n: LaunchConfiguration(n) for n in names}


def build(plant=None, defaults=None, mode='closed', bag_prefix='ibvs_'):
    """plant None declares a plant:= argument; mode 'observer' starts no gate and no
    controllers and records from launch."""
    if plant is not None and plant not in PLANTS:
        raise ValueError('plant %r is not one of %s' % (plant, ', '.join(PLANTS)))
    if mode not in ('closed', 'observer'):
        raise ValueError('mode %r is not closed or observer' % mode)
    presets = camera_presets()
    d = dict(DEPLOYMENT)
    d.update(defaults or {})
    closed = mode == 'closed'

    def arg(name, default, **kw):
        return DeclareLaunchArgument(name, default_value=d.get(name, default), **kw)

    args = [
        arg('headless', 'false'),
        # Must match <world name=...> in quad_gz_sim/worlds/ibvs.sdf.
        arg('world', 'ibvs'),
        arg('rosbag', 'false'),
        arg('foxglove', 'false'),
    ]
    if plant is None:
        args.append(arg('plant', 'analytic', choices=list(PLANTS)))
    if closed:
        args += [
            # false leaves the controllers unstarted, for open-loop testing.
            arg('controllers', 'true'),
            # launch: record the observer's transient too; handover: start at the gate.
            arg('record_from', 'handover', choices=['handover', 'launch']),
            # PX4's offboard-loss Hold, which these plants otherwise lack; false for A/B only.
            arg('sim_hold', 'true', choices=['true', 'false']),
        ]

    args += [
        # Plant. Mass reaches the plant, fixed_eso and pos_ctrl; the plant's inertia scales with it.
        arg('quad_mass', '1.30'),
        # Where the aircraft starts, over the target. Above zD it descends onto it.
        arg('start_altitude', '1.5'),
    ]
    if plant in (None, 'gazebo'):
        # gazebo only. Debug: false feeds a wrapped attitude, as a quaternion source would.
        args.append(arg('unwrap_attitude', 'true'))

    args += [
        # Camera and target; see quad_description/config/cameras.yaml.
        arg('camera', presets.DEFAULT_CAMERA),
        arg('target_scale', '0.5'),
        arg('marker_dict', '7x7'),
        # Above zero, renders at this rate instead of the SDF's 50.
        arg('camera_rate', '0.0'),
        # Detector; same knobs and defaults as sitl.launch.py.
        arg('detector_backend', 'hybrid'),
        arg('nano_error_correction', '0.3'),
        arg('nano_border_error_rate', '0.35'),
        arg('nano_box_filter', '15'),
        arg('nano_max_revisited', '0.05'),
        arg('roi_margin', '0.5'),
        arg('use_aruco3_detection', 'false'),
        arg('aruco3_margin', '0.7'),
        arg('cv_num_threads', '1'),

        # Disturbance; magnitudes in quad_gz_sim/config/disturbances.yaml. Profiles compose.
        arg('disturbance', 'none'),
        arg('disturbance_seed', '0'),
        arg('gust_scale', '1.0'),
        arg('wind_scale', '1.0'),
        arg('gust_tau', '1.5'),
        # table52 only: scales the Von Karman sigmas, leaving the mean schedule alone.
        arg('turbulence_scale', '1.0'),
        # m/s per axis, scaled by wind_scale. The default points down the camera's tight axis.
        arg('wind_velocity', '[0.8, 0.4, 0.0]'),

        # Target trajectory; see quad_gz_sim/scenario.launch.py.
        arg('target_profile', 'hover',
            description='thesis | hover | line | circle | steps'),
        arg('target_speed', '1.0'),
        arg('target_yaw_rate', '0.1'),
        arg('target_accel', '0.5'),
        # Degrees from +x. 0 = tight FOV axis; 90 = wide axis.
        arg('target_heading', '0.0'),
        # Holds the target until pos_ctrl publishes; target_release needs it. Without it a moving
        # target leaves at t = 0 and the gate's hold hands over to an aircraft already behind.
        arg('hold_target', 'true', choices=['true', 'false']),
        arg('target_release', 'control', description='control | settled'),
        # Hides the target for a window to exercise the lock-loss path. 0 disables.
        arg('blackout_at', '0.0'),
        arg('blackout_for', '3.0'),
        # Servoing depth. aD follows from it and from target_scale.
        arg('zD', '1.2'),

        # Observer. The control law is the object of study: these sweep it, not fix it.
        arg('gamma1_xy', '18.0'),
        arg('gamma2_xy', '20.0'),
        arg('gamma3_xy', '4.0'),
        arg('gamma3_yaw', '3.0'),
        arg('gamma1_yaw', '5.0'),
        arg('gamma2_yaw', '16.0'),
        arg('alpha_yaw', '0.75'),
        arg('beta_yaw', '1.2'),
        arg('gamma4_yaw', '0.001'),
        # 1.0 is as-flown; -1.0 is thesis Eq. 5.81's sign.
        arg('eso_yaw_sign', '1.0'),
        # Above zero, places fixed_eso's x/y gains as a triple pole at this rate.
        arg('observer_omega', '0.0'),
        # Seeded estimation error (qx,qy,qz,qpsi), for the fixed-time sweep.
        arg('initial_estimate_offset', '[0.0, 0.0, 0.0, 0.0]'),
        # Empty = zD. A deliberate mismatch is an A/B arm, not a setting.
        arg('eso_z_des', ''),
    ]
    if closed:
        args += [
            # ekf2 here reads the plant's own quad_velocity_BF, i.e. truth.
            arg('velocity_source', 'td', choices=['td', 'ekf2', 'vision', 'off']),
            arg('print_error', 'false'),
            # ibvs_gate; the defaults are the node's own.
            arg('max_feature_error', '0.15'),
            arg('estimators_ready', '5.0'),
            arg('gate_timeout', '120.0'),
        ]

    def plant_of(context):
        return plant or LaunchConfiguration('plant').perform(context).lower()

    def f(name, context):
        return float(LaunchConfiguration(name).perform(context))

    def _simulation(context, *a, **k):
        p = plant_of(context)
        plant_args = {'plant': p, 'quad_mass': LaunchConfiguration('quad_mass'),
                      'start_altitude': LaunchConfiguration('start_altitude')}
        if p == 'gazebo':
            plant_args['unwrap_attitude'] = LaunchConfiguration('unwrap_attitude')
        return [
            include(SIM_PKG, 'gz_sim.launch.py',
                    dict(_lc(('headless', 'world', 'camera', 'target_scale', 'marker_dict',
                              'camera_rate', 'quad_mass', 'start_altitude')), plant=p)),
            include(SIM_PKG, 'plant.launch.py', plant_args),
            include(SIM_PKG, 'scenario.launch.py', _lc(SCENARIO_ARGS)),
        ]

    def _estimation(context, *a, **k):
        # Resolved, not substituted: image_features takes intrinsics and aD, and aD is
        # computed from the preset, target_scale and zD.
        zD = f('zD', context)
        scale = f('target_scale', context)
        cam = presets.resolve(LaunchConfiguration('camera').perform(context), scale, zD)
        z_des = LaunchConfiguration('eso_z_des').perform(context).strip()
        return [
            LogInfo(msg=presets.summary(cam, zD, f('camera_rate', context))),
            include(CTRL_PKG, 'estimation.launch.py',
                    dict(_lc(ESTIMATION_ARGS),
                         z_des='%.6f' % (float(z_des) if z_des else zD),
                         camera_hfov='%.9f' % cam['hfov'],
                         camera_width=str(cam['width']),
                         camera_height=str(cam['height']),
                         camera_distortion=str(cam['distortion']),
                         aD='%.12g' % cam['aD'],
                         min_marker_length_ratio='%.6f' % presets.min_marker_ratio(
                             cam, scale, zD, f('aruco3_margin', context)),
                         camera_topic=('/quad/camera/image_distorted'
                                       if any(cam['distortion'])
                                       else '/quad/camera/image_raw'))),
        ]

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        # mcap, not rosbag2's default sqlite3: Foxglove Studio cannot open .db3.
        out = os.path.join('bags', bag_prefix + time.strftime('%Y%m%d_%H%M%S'))
        return [ExecuteProcess(
            # The recorded topics carry no header stamps, so the recorder's clock is the only
            # time axis the analysis has; it must be sim time.
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '-s', 'mcap', '-o', out]
                + bag_topics(),
            output='screen')]

    common = [
        OpaqueFunction(function=_simulation),
        OpaqueFunction(function=_estimation),
        include(PKG, 'viz.launch.py', _lc(('foxglove', 'camera'))),
    ]

    if not closed:
        return LaunchDescription(args + common + [OpaqueFunction(function=_bag)])

    def from_launch(context):
        return LaunchConfiguration('record_from').perform(context).lower() == 'launch'

    def _bag_at_launch(context, *a, **k):
        return _bag(context) if from_launch(context) else []

    def _sim_hold(context, *a, **k):
        if LaunchConfiguration('sim_hold').perform(context).lower() != 'true':
            return []
        return [Node(package=SIM_PKG, executable='sim_hold', name='sim_hold', output='screen',
                     parameters=[{'use_sim_time': True,
                                  'quad_mass': ParameterValue(
                                      LaunchConfiguration('quad_mass'), value_type=float)}])]

    # Exits 0 once closed-loop servoing is possible, non-zero if it never is. A Node, not an
    # include: the event handler below has to hold the action object.
    gate = Node(package=CTRL_PKG, executable='ibvs_gate', name='ibvs_gate',
                output='screen',
                parameters=[{'use_sim_time': True,
                             'max_feature_error': ParameterValue(
                                 LaunchConfiguration('max_feature_error'), value_type=float),
                             'estimators_ready': ParameterValue(
                                 LaunchConfiguration('estimators_ready'), value_type=float),
                             'timeout': ParameterValue(
                                 LaunchConfiguration('gate_timeout'), value_type=float)}])

    # Do NOT go back to a TimerAction: that delay is wall clock while these run on sim time.
    control = RegisterEventHandler(OnProcessExit(
        target_action=gate,
        on_exit=lambda event, context: (
            ([include(CTRL_PKG, 'control.launch.py',
                      _lc(('zD', 'quad_mass', 'velocity_source', 'print_error')))]
             if LaunchConfiguration('controllers').perform(context).lower() == 'true'
             else [LogInfo(msg='controllers:=false - plant left open-loop.')])
            + ([] if from_launch(context) else [OpaqueFunction(function=_bag)])
            if event.returncode == 0 else
            [LogInfo(msg='ibvs_gate failed - controllers not started. See its error above.')]
        )))

    return LaunchDescription(
        args + [OpaqueFunction(function=_bag_at_launch)] + common
        + [OpaqueFunction(function=_sim_hold), control, gate])
