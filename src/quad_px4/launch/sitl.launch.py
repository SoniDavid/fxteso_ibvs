"""FxTESO + ANFTIBVS on top of PX4 SITL. The full stack with PX4 as the plant.

  quad_gz_sim/gz_sim (plant:=px4) -> world, ArUco target, camera bridge, /clock
        |                                              |
        |                            PX4 SITL (standalone) attaches to the F450
        |                                   already in that world
        |                                              |
        |                              MicroXRCEAgent  <-> uXRCE-DDS
        v                                              |
  quad_gz_sim/scenario -> tgt_* and /disturbances      v
                                              px4_state_adapter
                                    quad_position / quad_attitude / ...
                                                       |
  quad_control/estimation: td_linear, td_attitude, td_attitude_desired, image_features
                                                       |
                        fixed_eso -> pos_ctrl -> px4_offboard_bridge -> /fmu/in/*

PX4 owns allocation, attitude and rates, so att_ctrl does not run here. Everything from
image_features through pos_ctrl is the same code the analytic and gazebo plants use.

Start-up runs through two gates:

  px4_takeoff_gate   aircraft settled at the servoing altitude -> start the estimators
  ibvs_gate          plant, target and a held marker lock      -> start pos_ctrl

  ros2 launch quad_px4 sitl.launch.py [headless:=true] [rosbag:=true] [foxglove:=true]
                                      [controllers:=false] [record_from:=handover|launch]
                                      [disturbance:=none|step|gust|wind|table52|csv]
                                      [disturbance_seed:=N] [turbulence_scale:=1.0]
                                      [px4_dir:=...] [xrce_agent:=...]

Defaults: module3wide_2304, takeoff at 1.5 m, servo at zD 1.2 with a 0.5-scale target, holding
station. See RUNS.md at the repository root for the standing configurations.

Prerequisites, both one-off:
  git submodule update --init --recursive external/PX4-Autopilot
  make -C external/PX4-Autopilot px4_sitl_default
"""
import importlib.util
import math
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'quad_px4'
SIM_PKG = 'quad_gz_sim'
CTRL_PKG = 'quad_control'
UTILS_PKG = 'quad_utils'

# Must match <world name=...> in quad_gz_sim/worlds/ibvs.sdf and the <name> of the F450
# include in it: PX4 addresses both by name when it attaches.
WORLD = 'ibvs'
MODEL = 'F450'
# The airframe id of ROMFS/px4fmu_common/init.d-posix/airframes/22100_gz_F450_px4 in the
# PX4 fork. 22100 sits in PX4's reserved [22000, 22999] custom-model range.
SYS_AUTOSTART = '22100'

# Rotor model fitted to T-Motor's bench table for the MN2212 V2.0 KV920 on a 9545B.
# Only the 9545 is measured; any other entry is an estimate until it is benched.
PROPS = {
    '9545': (8.566e-06, 702.7),   # measured
}
MASS, GRAVITY = 2.0, 9.81
ROTOR_MIN = 150.0                 # SIM_GZ_EC_MIN, the ESC idle floor


def rotor_setup(prop, cells):
    """Rotor velocity ceiling and hover throttle for a prop and cell count.

    PX4 maps its normalised output linearly onto [SIM_GZ_EC_MIN, SIM_GZ_EC_MAX] rotor velocity
    while thrust goes as omega^2, so hover throttle is not mg/T_max. This expression reproduced
    the previously measured MPC_THR_HOVER of 0.716 to 0.001.
    """
    c_t, kv_loaded = PROPS[prop]
    w_max = kv_loaded * cells * 3.7 * 2.0 * math.pi / 60.0
    w_hover = math.sqrt((MASS * GRAVITY / 4.0) / c_t)
    if w_hover >= w_max:
        raise RuntimeError('%s on %dS cannot hover %.1f kg.' % (prop, cells, MASS))
    return w_max, (w_hover - ROTOR_MIN) / (w_max - ROTOR_MIN)


# Spawn pose from worlds/ibvs.sdf, in NED (the world is ENU, so y and z are negated). EKF2
# anchors its local frame there; px4_state_adapter adds this to get back to the world frame.
# Directly over the target: PX4 loiter holds the takeoff point, and qx ~ dx/zD, so the old
# 0.14 m offset read 0.06 at zD 2.5 but 0.17 at zD 1.2 - outside ibvs_gate's 0.15 limit.
SPAWN_NED = (-10.0, -10.0, -0.1)

# PX4's takeoff settles below its own setpoint by a repeatable amount - 1.10/1.11/1.11 m
# measured against a 1.20 m command. Added to MIS_TAKEOFF_ALT only, so `takeoff_alt` means the
# altitude actually reached and px4_takeoff_gate can hold a tight tolerance around it.
TAKEOFF_UNDERSHOOT = 0.095

# EKF2's North is Gazebo +y (the world is ENU) while the workspace calls +x North.
FRAME_YAW_OFFSET = -math.pi / 2.0


def camera_presets():
    """quad_gz_sim's camera preset resolver - the one implementation of the intrinsics and
    the aD derivation. share/<pkg>/launch is not on sys.path, so it is loaded by path."""
    path = os.path.join(get_package_share_directory(SIM_PKG), 'launch', 'camera_presets.py')
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
    """Flatten quad_utils' config/bag_topics.yaml, which groups topics by their source."""
    path = os.path.join(get_package_share_directory(UTILS_PKG), 'config', 'bag_topics.yaml')
    with open(path) as fh:
        groups = yaml.safe_load(fh)
    return [topic for group in groups.values() for topic in group]


def generate_launch_description():
    presets = camera_presets()
    args = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('controllers', default_value='true'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('disturbance', default_value='none'),
        DeclareLaunchArgument('disturbance_seed', default_value='0'),
        # Scale the magnitudes in quad_gz_sim/config/disturbances.yaml; one number per sweep
        # point, with the YAML keeping the shape.
        DeclareLaunchArgument('gust_scale', default_value='1.0'),
        DeclareLaunchArgument('wind_scale', default_value='1.0'),
        DeclareLaunchArgument('gust_tau', default_value='1.5'),
        # table52 only: scales the Von Karman sigmas, leaving the mean schedule alone.
        DeclareLaunchArgument('turbulence_scale', default_value='1.0'),
        # Target trajectory; see quad_gz_sim/scenario.launch.py. 'hover' pairs with the default
        # depth below; the 'thesis' course wants the 2.5 m geometry - see RUNS.md.
        DeclareLaunchArgument('target_profile', default_value='hover'),
        DeclareLaunchArgument('target_speed', default_value='1.0'),
        DeclareLaunchArgument('target_yaw_rate', default_value='0.1'),
        DeclareLaunchArgument('target_accel', default_value='0.5'),
        # Above zero, places fixed_eso's x/y gains as a triple pole at this rate.
        DeclareLaunchArgument('observer_omega', default_value='0.0'),
        # 4S, not 3S: at 2.0 kg a 9545 on 3S needs 91% throttle to hover, which leaves the
        # attitude loop nothing. On 4S the same prop gives T/W 2.07 and hovers at 0.65.
        DeclareLaunchArgument('battery_cells', default_value='4'),
        DeclareLaunchArgument('prop', default_value='9545'),
        # No per-plant value: at n=8 against n=6, 18 and 9 are indistinguishable on px4.
        DeclareLaunchArgument('gamma1_xy', default_value='18.0'),
        # Table 5.3 has gamma2_xy 10, gamma3_xy 7, gamma3_yaw 7.
        DeclareLaunchArgument('gamma2_xy', default_value='20.0'),
        DeclareLaunchArgument('gamma3_xy', default_value='4.0'),
        DeclareLaunchArgument('gamma3_yaw', default_value='3.0'),
        DeclareLaunchArgument('gamma1_yaw', default_value='5.0'),
        DeclareLaunchArgument('gamma2_yaw', default_value='16.0'),
        DeclareLaunchArgument('alpha_yaw', default_value='0.75'),
        DeclareLaunchArgument('beta_yaw', default_value='1.2'),
        DeclareLaunchArgument('gamma4_yaw', default_value='0.001'),
        # 1.0 is the as-flown observer yaw sign; -1.0 is thesis Eq. 5.81's -1.
        DeclareLaunchArgument('eso_yaw_sign', default_value='1.0'),
        # Seeded initial estimation error (qx,qy,qz,qpsi) added to fixed_eso's initial state.
        # Sweeping it is how the fixed-time claim gets measured: settling time must stay
        # bounded as this grows. Available on estimation.launch.py; exposed here so it can be
        # flown against the px4 plant rather than only the analytic one.
        DeclareLaunchArgument('initial_estimate_offset', default_value='[0.0, 0.0, 0.0, 0.0]'),
        # The observer's z_d (thesis Eq. 5.81), which sets g_xi = -1/z_d on x/y/z. Empty means
        # "use zD", which is what it must be: a mismatch leaves part of the command booked as
        # disturbance and fed back in. 2.5 reproduces every run flown before 2026-09-04, when
        # this was frozen at 2.5 regardless of zD - that is the A/B arm, not a setting to fly.
        DeclareLaunchArgument('eso_z_des', default_value=''),
        # How aligned the markers must be before pos_ctrl takes over. |qpsi| at
        # handover separates held from collapsed runs at ~0.17; 0 disables the test.
        DeclareLaunchArgument('max_feature_error', default_value='0.15'),
        # Settled hover held before handover. EKF2's pitch reads ~0.019 rad high for the
        # first 10 s and decays through zero by ~25 s; commanding 0 against that bias is
        # 0.18 m/s^2 of uncommanded forward acceleration, straight into the tight FOV axis.
        DeclareLaunchArgument('estimators_ready', default_value='5.0'),
        # ibvs_gate's own default. Raise it when the aircraft needs longer to align: PX4
        # latches its yaw setpoint against an EKF2 heading that has not finished aligning, so
        # the true yaw swings tens of degrees during takeoff and unwinds over ~100 s. The
        # target is held until handover, so waiting costs wall clock and nothing else.
        DeclareLaunchArgument('gate_timeout', default_value='120.0'),
        # EKF2 only fuses mag heading while horizontal acceleration exceeds this and GNSS is
        # aiding. 0.0 keeps heading aided through station-keeping; PX4's default is 0.5.
        # Exposed so the two can be A/B'd without a rebuild.
        DeclareLaunchArgument('mag_acclim', default_value='0.0'),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.path.expanduser(
                '~/Robotics/fxteso_ibvs/external/PX4-Autopilot'),
            description='The PX4 fork submodule, holding build/px4_sitl_default.'),
        DeclareLaunchArgument(
            'xrce_agent',
            default_value=os.path.expanduser(
                '~/Robotics/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'),
            description='MicroXRCEAgent binary. It is not normally on PATH.'),
        # Anchors the newton -> normalized thrust map; keep equal to MPC_THR_HOVER in the
        # fork's airframes/22100_gz_F450_px4, which documents how it was measured.
        DeclareLaunchArgument('hover_thrust', default_value='0.6461'),
        # Camera module and sensor mode; see quad_gz_sim/config/cameras.yaml. Feeds both the
        # rendered <camera> block and image_features' intrinsics, from one table.
        DeclareLaunchArgument('camera', default_value=presets.DEFAULT_CAMERA),
        # 0.5 is the printed target, 450 x 375 mm - the sheet that exists, not a tuned value.
        DeclareLaunchArgument('target_scale', default_value='0.5'),
        DeclareLaunchArgument('marker_dict', default_value='7x7'),
        DeclareLaunchArgument('camera_rate', default_value='0.0'),
        # Servoing depth. aD follows from it and target_scale, and MIS_TAKEOFF_ALT is pushed
        # into PX4 to match - otherwise takeoff delivers the aircraft to the wrong depth and
        # the feature vector is mis-scaled from the first frame.
        # Servoing depth, and the tighter of the two geometries flown: the field-of-view budget
        # and the station-keeping margin both shrink with it.
        DeclareLaunchArgument('zD', default_value='1.2'),
        # Depth at handover. Empty means "use zD". 1.5 against zD 1.2 is deliberate: take off
        # high and descend onto the target rather than climb to it. Empty gives no depth error.
        DeclareLaunchArgument('takeoff_alt', default_value='1.5'),
        # How close to takeoff_alt the gate insists on. The node's own 0.5 m default is wide
        # enough to straddle the depth stability threshold, making the IC an accident.
        DeclareLaunchArgument('takeoff_tolerance', default_value='0.10'),
        # Where the recorder starts. 'handover' records the servoing run and nothing before it,
        # which is what every flight wants. 'launch' is for measuring the OBSERVER: fixed_eso
        # starts at the takeoff gate and converges in ~1 s, so a recorder started at handover -
        # or even at the gate, since it needs ~1 s to come up - misses the transient entirely.
        DeclareLaunchArgument('record_from', default_value='handover',
                              description='handover | launch'),
    ]

    def takeoff_alt_of(context):
        """takeoff_alt, defaulting to zD. One resolver so PX4 and the gate cannot disagree."""
        raw = LaunchConfiguration('takeoff_alt').perform(context).strip()
        return float(raw) if raw else float(LaunchConfiguration('zD').perform(context))

    def eso_z_des_of(context):
        """eso_z_des, defaulting to zD - the same idiom as takeoff_alt_of, for the same reason."""
        raw = LaunchConfiguration('eso_z_des').perform(context).strip()
        return float(raw) if raw else float(LaunchConfiguration('zD').perform(context))

    simulation = [
        include(SIM_PKG, 'gz_sim.launch.py',
                {'headless': LaunchConfiguration('headless'),
                 'world': WORLD,
                 'plant': 'px4',
                 'camera': LaunchConfiguration('camera'),
                 'target_scale': LaunchConfiguration('target_scale'),
                 'marker_dict': LaunchConfiguration('marker_dict'),
                 'camera_rate': LaunchConfiguration('camera_rate')}),
        include(SIM_PKG, 'scenario.launch.py',
                {'disturbance': LaunchConfiguration('disturbance'),
                 'disturbance_seed': LaunchConfiguration('disturbance_seed'),
                 'gust_scale': LaunchConfiguration('gust_scale'),
                 'wind_scale': LaunchConfiguration('wind_scale'),
                 'gust_tau': LaunchConfiguration('gust_tau'),
                 'turbulence_scale': LaunchConfiguration('turbulence_scale'),
                 # Arming and takeoff cost sim time the other plants do not spend; without
                 # this the target leaves the camera footprint before ibvs_gate can lock.
                 'hold_target': 'true',
                 'target_profile': LaunchConfiguration('target_profile'),
                 'target_speed': LaunchConfiguration('target_speed'),
                 'target_yaw_rate': LaunchConfiguration('target_yaw_rate'),
                 'target_accel': LaunchConfiguration('target_accel'),
                 'zD': LaunchConfiguration('zD')}),
    ]

    def _px4(context, *a, **k):
        """PX4 SITL in standalone mode, plus the uXRCE-DDS agent that fronts it for ROS 2."""
        px4_dir = LaunchConfiguration('px4_dir').perform(context)
        agent = LaunchConfiguration('xrce_agent').perform(context)

        binary = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
        rootfs = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs')
        if not os.path.isfile(binary):
            raise RuntimeError(
                '%s not found. Build PX4 first:  make -C %s px4_sitl_default' % (binary, px4_dir))
        if not os.path.isfile(agent):
            raise RuntimeError(
                '%s not found. Pass xrce_agent:=<path to MicroXRCEAgent>.' % agent)

        # STANDALONE stops PX4 starting its own gz server; MODEL_NAME binds it to the F450
        # this world already contains. See PX4's init.d-posix/px4-rc.gzsim.
        env = dict(os.environ)
        env.update({
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': WORLD,
            'PX4_GZ_MODEL_NAME': MODEL,
            'PX4_SYS_AUTOSTART': SYS_AUTOSTART,
            'PX4_GZ_NO_FOLLOW': '1',   # the world's own camera pose is the framing we want
        })

        # Rotor ceiling and hover anchor are one derivation; PX4 must not carry a second copy.
        # Keep model.sdf's maxRotVelocity above these or the motor model clamps silently.
        prop = LaunchConfiguration('prop').perform(context)
        cells = int(LaunchConfiguration('battery_cells').perform(context))
        w_max, thr_hover = rotor_setup(prop, cells)
        for i in (1, 2, 3, 4):
            env['PX4_PARAM_SIM_GZ_EC_MAX%d' % i] = '%d' % round(w_max)
        env['PX4_PARAM_MPC_THR_HOVER'] = '%.4f' % thr_hover

        # Loss of marker lock stops the setpoint stream. The default 0 = Position expects RC
        # that SITL has not, so the aircraft descends; 5 = Hold is the only recoverable mode.
        env['PX4_PARAM_COM_OBL_RC_ACT'] = '5'

        # Takeoff has to deliver the aircraft to the depth image_features' aD was computed
        # for, or the feature vector is mis-scaled from the first frame. Same derivation, one
        # argument: the airframe's own default is only for a bare px4_sitl_default run.
        env['PX4_PARAM_MIS_TAKEOFF_ALT'] = '%.3f' % (takeoff_alt_of(context) + TAKEOFF_UNDERSHOOT)

        env['PX4_PARAM_EKF2_MAG_ACCLIM'] = '%.3f' % float(
            LaunchConfiguration('mag_acclim').perform(context))

        # px4_offboard_bridge maps newtons to normalised thrust with its own hover_thrust, so a
        # drift between the two silently mis-scales every command. Fail loudly instead.
        declared = float(LaunchConfiguration('hover_thrust').perform(context))
        if abs(declared - thr_hover) > 0.005:
            raise RuntimeError(
                'hover_thrust:=%.4f but %s on %dS derives %.4f. Pass hover_thrust:=%.4f.'
                % (declared, prop, cells, thr_hover, thr_hover))

        return [
            ExecuteProcess(cmd=[agent, 'udp4', '-p', '8888'],
                           name='micro_xrce_agent', output='log'),
            # px4-rc.gzsim already polls 30 s for the world; this just keeps the console clean.
            TimerAction(period=5.0, actions=[
                ExecuteProcess(cmd=[binary], cwd=rootfs, env=env,
                               name='px4_sitl', output='screen'),
            ]),
        ]

    state_adapter = Node(
        package=PKG, executable='px4_state_adapter', name='px4_state_adapter',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'origin_north': SPAWN_NED[0],
                     'origin_east': SPAWN_NED[1],
                     'origin_down': SPAWN_NED[2],
                     'frame_yaw_offset': FRAME_YAW_OFFSET}])

    def _offboard_bridge(context, *a, **k):
        # Must come from the SAME resolver as MIS_TAKEOFF_ALT and the takeoff gate: left at the
        # node's own default the bridge never streams below it, and records loiter as servoing.
        return [Node(
            package=PKG, executable='px4_offboard_bridge', name='px4_offboard_bridge',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'hover_thrust': LaunchConfiguration('hover_thrust'),
                         'takeoff_altitude': takeoff_alt_of(context),
                         # Same value as the adapter's: one rotates into the workspace frame,
                         # the other rotates back out of it.
                         'frame_yaw_offset': FRAME_YAW_OFFSET}])]

    viz = include(UTILS_PKG, 'viz.launch.py', {'foxglove': LaunchConfiguration('foxglove'),
                                               'camera': LaunchConfiguration('camera')})

    def _bag_at_launch(context, *a, **k):
        """record_from:=launch. Only the observer experiments need this; see the argument."""
        if LaunchConfiguration('record_from').perform(context).lower() != 'launch':
            return []
        return _bag(context)

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        # mcap, not rosbag2's default sqlite3: Foxglove Studio cannot open .db3.
        out = os.path.join('bags', 'px4_' + time.strftime('%Y%m%d_%H%M%S'))
        return [ExecuteProcess(
            # --use-sim-time: almost every recorded topic is a bare Vector3/Quaternion/
            # Float64 with no header stamp, so the recorder's own clock IS the time axis.
            # Without this it is the wall clock, and headless SITL does not hold RTF 1.0.
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '-s', 'mcap', '-o', out]
                + bag_topics(),
            output='screen')]

    # px4_takeoff_gate -> estimation + ibvs_gate -> pos_ctrl. Its altitude must track zD or the
    # gate times out. A plain Node, not an OpaqueFunction: OnProcessExit needs the action object.
    takeoff_gate = Node(
        package=PKG, executable='px4_takeoff_gate', name='px4_takeoff_gate', output='screen',
        parameters=[{'use_sim_time': True,
                     'altitude': ParameterValue(
                         PythonExpression(["'", LaunchConfiguration('takeoff_alt'),
                                           "' or '", LaunchConfiguration('zD'), "'"]),
                         value_type=float),
                     'tolerance': ParameterValue(
                         LaunchConfiguration('takeoff_tolerance'), value_type=float)}])

    # Same gate as the other plants: exits 0 once the aircraft is placed, the target is
    # placed and the markers have held a lock.
    gate = Node(package=CTRL_PKG, executable='ibvs_gate', name='ibvs_gate',
                output='screen',
                parameters=[{'use_sim_time': True,
                             'max_feature_error': ParameterValue(
                                 LaunchConfiguration('max_feature_error'), value_type=float),
                             'estimators_ready': ParameterValue(
                                 LaunchConfiguration('estimators_ready'), value_type=float),
                             'timeout': ParameterValue(
                                 LaunchConfiguration('gate_timeout'), value_type=float)}])

    # pos_ctrl publishing desired_attitude is what tips px4_offboard_bridge into OFFBOARD,
    # so starting it here is the handover. att_ctrl stays off: PX4 owns the inner loop.
    control = RegisterEventHandler(OnProcessExit(
        target_action=gate,
        on_exit=lambda event, context: (
            ([include(CTRL_PKG, 'control.launch.py',
                      {'attitude_controller': 'false', 'zD': LaunchConfiguration('zD')})]
             if LaunchConfiguration('controllers').perform(context).lower() == 'true'
             else [LogInfo(msg='controllers:=false - the aircraft will loiter after takeoff.')])
            + ([OpaqueFunction(function=_bag)]
               if LaunchConfiguration('record_from').perform(context).lower() != 'launch'
               else [])
            if event.returncode == 0 else
            [LogInfo(msg='ibvs_gate failed - controllers not started. See its error above.')]
        )))

    def _estimation_include(context):
        # Resolved, not substituted: image_features takes intrinsics and aD, and aD is
        # computed from the preset, target_scale and zD.
        cam = presets.resolve(
            LaunchConfiguration('camera').perform(context),
            float(LaunchConfiguration('target_scale').perform(context)),
            float(LaunchConfiguration('zD').perform(context)))
        return [
            LogInfo(msg=presets.summary(
                cam, float(LaunchConfiguration('zD').perform(context)),
                float(LaunchConfiguration('camera_rate').perform(context)))),
            include(CTRL_PKG, 'estimation.launch.py',
                    {'initial_estimate_offset': LaunchConfiguration('initial_estimate_offset'),
                     'z_des': '%.6f' % eso_z_des_of(context),
                     'gamma1_xy': LaunchConfiguration('gamma1_xy'),
                     'gamma2_xy': LaunchConfiguration('gamma2_xy'),
                     'gamma3_xy': LaunchConfiguration('gamma3_xy'),
                     'gamma3_yaw': LaunchConfiguration('gamma3_yaw'),
                     'gamma1_yaw': LaunchConfiguration('gamma1_yaw'),
                     'gamma2_yaw': LaunchConfiguration('gamma2_yaw'),
                     'alpha_yaw': LaunchConfiguration('alpha_yaw'),
                     'beta_yaw': LaunchConfiguration('beta_yaw'),
                     'gamma4_yaw': LaunchConfiguration('gamma4_yaw'),
                     'eso_yaw_sign': LaunchConfiguration('eso_yaw_sign'),
                     'observer_omega': LaunchConfiguration('observer_omega'),
                     'camera_hfov': '%.9f' % cam['hfov'],
                     'camera_width': str(cam['width']),
                     'camera_height': str(cam['height']),
                     'camera_distortion': str(cam['distortion']),
                     'aD': '%.12g' % cam['aD'],
                     'marker_dict': LaunchConfiguration('marker_dict'),
                     'camera_topic': ('/quad/camera/image_distorted'
                                      if any(cam['distortion'])
                                      else '/quad/camera/image_raw')}),
        ]

    estimation = RegisterEventHandler(OnProcessExit(
        target_action=takeoff_gate,
        on_exit=lambda event, context: (
            _estimation_include(context) + [gate, control]
            if event.returncode == 0 else
            # Tear the run down instead of idling to the harness timeout. EKF2 never recovers
            # from a failed initialisation - one run sat 82 s - so the remaining minutes buy
            # nothing, and in a sweep they are the single largest cost.
            [LogInfo(msg='px4_takeoff_gate failed - estimators not started. See its error '
                         'above.'), Shutdown(reason='takeoff gate failed')]
        )))

    return LaunchDescription(
        args + [OpaqueFunction(function=_bag_at_launch)] + simulation
        + [OpaqueFunction(function=_px4), state_adapter,
           OpaqueFunction(function=_offboard_bridge), viz,
           estimation, takeoff_gate])
