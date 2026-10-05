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
station, in the Vicon lab with EKF2 aided by mocap pose all flight (venue:=vicon
aiding_policy:=always velocity_source:=off). See RUNNING.md at the repository root for the
standing configurations.

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
                            RegisterEventHandler, SetLaunchConfiguration, Shutdown,
                            TimerAction)
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
GRAVITY = 9.81
ROTOR_MIN = 150.0                 # SIM_GZ_EC_MIN, the ESC idle floor
# MPC_THR_HOVER's documented maximum; PositionControl clamps hover thrust at 0.9 regardless.
THR_HOVER_MAX = 0.8


def rotor_setup(prop, cells, mass):
    """Rotor velocity ceiling, hover throttle and T/W for a prop, cell count and mass.

    PX4 maps its normalised output linearly onto [SIM_GZ_EC_MIN, SIM_GZ_EC_MAX] rotor velocity
    while thrust goes as omega^2, so hover throttle is not mg/T_max. This expression reproduced
    the previously measured MPC_THR_HOVER of 0.716 to 0.001.
    """
    c_t, kv_loaded = PROPS[prop]
    weight = mass * GRAVITY
    w_max = kv_loaded * cells * 3.7 * 2.0 * math.pi / 60.0
    w_hover = math.sqrt((weight / 4.0) / c_t)
    thr_hover = (w_hover - ROTOR_MIN) / (w_max - ROTOR_MIN)
    t_w = 4.0 * c_t * w_max * w_max / weight
    if thr_hover > THR_HOVER_MAX:
        raise RuntimeError(
            '%s on %dS hovers %.2f kg at %.2f throttle (T/W %.2f); PX4 allows at most %.1f. '
            'Check quad_mass, or add cells.' % (prop, cells, mass, thr_hover, t_w, THR_HOVER_MAX))
    return w_max, thr_hover, t_w


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
    """quad_description's camera preset resolver - the one implementation of the intrinsics and
    the aD derivation. share/<pkg>/launch is not on sys.path, so it is loaded by path."""
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
        DeclareLaunchArgument('gust_scale', default_value='1.0'),
        DeclareLaunchArgument('wind_scale', default_value='1.0'),
        DeclareLaunchArgument('gust_tau', default_value='1.5'),
        DeclareLaunchArgument('turbulence_scale', default_value='1.0'),
        # 'thesis' wants the 2.5 m geometry — see RUNNING.md.
        DeclareLaunchArgument('target_profile', default_value='hover'),
        DeclareLaunchArgument('target_speed', default_value='1.0'),
        DeclareLaunchArgument('target_yaw_rate', default_value='0.1'),
        DeclareLaunchArgument('target_accel', default_value='0.5'),
        # Above zero, places fixed_eso's x/y gains as a triple pole at this rate.
        DeclareLaunchArgument('observer_omega', default_value='0.0'),
        # The 3S build. The thesis plant (2.0 kg) needs 4S: on 3S it hovers at 91% and is refused.
        DeclareLaunchArgument('battery_cells', default_value='3'),
        DeclareLaunchArgument('prop', default_value='9545'),
        # Parts-list estimate for the 3S build (1.19-1.37 kg), not yet weighed. Sets the SDF,
        # PX4, bridge and controllers. Thesis plant: quad_mass:=2.0 battery_cells:=4.
        DeclareLaunchArgument('quad_mass', default_value='1.30'),
        # pos_ctrl's per-cycle error print; off by default so the WARNs stay readable.
        DeclareLaunchArgument('print_error', default_value='false'),
        # Which velocity feeds pos_ctrl's Coriolis term - the ONE place EKF2 position reaches the
        # control output. off keeps Vicon-aided position out of the loop (~0.004 N dropped, E84);
        # td is what every bag before 2026-10-04 was flown with.
        DeclareLaunchArgument('velocity_source', default_value='off',
                              choices=['td', 'ekf2', 'vision', 'off']),
        # Table 5.3 thesis values: gamma2_xy=10, gamma3_xy=7, gamma3_yaw=7.
        DeclareLaunchArgument('gamma1_xy', default_value='18.0'),
        DeclareLaunchArgument('gamma2_xy', default_value='20.0'),
        DeclareLaunchArgument('gamma3_xy', default_value='4.0'),
        DeclareLaunchArgument('gamma3_yaw', default_value='3.0'),
        DeclareLaunchArgument('gamma1_yaw', default_value='5.0'),
        DeclareLaunchArgument('gamma2_yaw', default_value='16.0'),
        DeclareLaunchArgument('alpha_yaw', default_value='0.75'),
        DeclareLaunchArgument('beta_yaw', default_value='1.2'),
        DeclareLaunchArgument('gamma4_yaw', default_value='0.001'),
        # 1.0 is as-flown; -1.0 is thesis Eq. 5.81's sign.
        DeclareLaunchArgument('eso_yaw_sign', default_value='1.0'),
        # Seeded estimation error (qx,qy,qz,qpsi). Sweep to verify fixed-time convergence claim.
        DeclareLaunchArgument('initial_estimate_offset', default_value='[0.0, 0.0, 0.0, 0.0]'),
        # Empty = use zD. A mismatch books part of the command as disturbance and feeds it back.
        # Pass 2.5 to reproduce runs flown before 2026-09-04 (A/B arm only, not a normal setting).
        DeclareLaunchArgument('eso_z_des', default_value=''),
        # Known harmful: re-arms OFFBOARD off the heartbeat while the loop is still blind.
        DeclareLaunchArgument('offboard_recovery', default_value='false'),
        # |qpsi| at handover separates held from collapsed runs at ~0.17; 0 disables.
        DeclareLaunchArgument('max_feature_error', default_value='0.15'),
        # EKF2 pitch bias ~0.019 rad for first 10 s → 0.18 m/s² uncommanded acceleration.
        DeclareLaunchArgument('estimators_ready', default_value='5.0'),
        # PX4 yaw unwinds over ~100 s after takeoff; target is held so waiting costs only wall clock.
        DeclareLaunchArgument('gate_timeout', default_value='120.0'),
        # One arg drives every node so they cannot disagree. See RUNNING.md for full explanation.
        # vicon (the flight configuration) is everything indoor does, plus the EV params,
        # vicon_sim and the bridge. indoor is the unaided GPS-denied arm.
        DeclareLaunchArgument('venue', default_value='vicon',
                              choices=['outdoor', 'indoor', 'vicon']),
        # venue:=vicon only. vicon_sim's imitation of the lab; the bridge is the one that flies.
        DeclareLaunchArgument('vicon_latency', default_value='0.02'),
        DeclareLaunchArgument('vicon_noise_sd', default_value='0.001'),
        DeclareLaunchArgument('vicon_dropout_probability', default_value='0.0'),
        # EKF2_EV_DELAY, ms. 0: the bridge's timestamp_sample already carries the capture time, so
        # EKF2 subtracting vicon_latency again would date every sample that much too early.
        DeclareLaunchArgument('ev_delay', default_value='0.0'),
        # always: full pose (3D position + yaw) into EKF2 all flight - the flight configuration.
        # gated: horizontal position only, cut at the handover (the E85 A/B).
        DeclareLaunchArgument('aiding_policy', default_value='always',
                              choices=['gated', 'always', 'oneshot', 'manual']),
        # DIAGNOSTIC. venue:=vicon only. Feeds EKF2 a VELOCITY reference and no position one -
        # what an optical flow sensor supplies - so the flow route can be decided before any
        # hardware is bought. Every result showing aiding works (E85, E88) used mocap POSITION.
        DeclareLaunchArgument('ev_velocity', default_value='false',
                              choices=['true', 'false']),
        # EKF2_IMU_CTRL bitmask: 0 gyro bias, 1 accel bias, 2 gravity fusion. 7 is PX4's default
        # and leaves behaviour unchanged. Unaided, a tilt and a horizontal accel bias are
        # unobservable as a pair, so EKF2 trades them and both wander  (2.13 deg tilt error
        # indoors against 0.29 aided). 5 inhibits the accel-bias half of that trade.
        DeclareLaunchArgument('imu_ctrl', default_value='7'),
        # 0.0 keeps heading aided through station-keeping (PX4 default is 0.5).
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
        # Empty = derived from prop, battery_cells and quad_mass. A value given must agree with it.
        DeclareLaunchArgument('hover_thrust', default_value=''),
        # Feeds both the rendered <camera> block and image_features' intrinsics from one table.
        DeclareLaunchArgument('camera', default_value=presets.DEFAULT_CAMERA),
        # 0.5 = printed target, 450×375 mm — the sheet that exists, not a tuned value.
        DeclareLaunchArgument('target_scale', default_value='0.5'),
        DeclareLaunchArgument('marker_dict', default_value='7x7'),
        # nano | opencv | hybrid (nano while locked, opencv otherwise) | roi (opencv around the last
        # detection). aruco3 only applies to opencv.
        DeclareLaunchArgument('detector_backend', default_value='hybrid'),
        # aruco_nano tolerances (nano's own 0 / 0 rejects a marker for one mis-read bit).
        # roi_margin: fraction of the target box.
        DeclareLaunchArgument('nano_error_correction', default_value='0.3'),
        DeclareLaunchArgument('nano_border_error_rate', default_value='0.35'),
        DeclareLaunchArgument('nano_box_filter', default_value='15'),
        DeclareLaunchArgument('nano_max_revisited', default_value='0.05'),
        DeclareLaunchArgument('roi_margin', default_value='0.5'),
        DeclareLaunchArgument('use_aruco3_detection', default_value='false'),
        # Fraction of the nominal marker size aruco3 must still find.
        DeclareLaunchArgument('aruco3_margin', default_value='0.7'),
        # OpenCV threads; at 1 its idle pool stops burning CPU next to nano.
        DeclareLaunchArgument('cv_num_threads', default_value='1'),
        DeclareLaunchArgument('camera_rate', default_value='0.0'),
        # Degrees from +x. 0 = tight FOV axis; 90 = wide axis (~2× field budget).
        DeclareLaunchArgument('target_heading', default_value='0.0'),
        # control: the target moves on the loop's first command (every run before E96).
        # settled: it waits until the loop holds it centred at zD, as the lab pushes the cart.
        DeclareLaunchArgument('target_release', default_value='control',
                              description='control | settled'),
        # m/s per axis, scaled by wind_scale. Default points down the camera's tight axis.
        DeclareLaunchArgument('wind_velocity', default_value='[0.8, 0.4, 0.0]'),
        # Hides target to exercise lock-loss path. 0 disables.
        DeclareLaunchArgument('blackout_at', default_value='0.0'),
        DeclareLaunchArgument('blackout_for', default_value='3.0'),
        # Indoor stand-in pilot lets go in OFFBOARD; false flies through it and trips PX4's override.
        DeclareLaunchArgument('pilot_hands_off', default_value='true'),
        # Seconds its hands stay still after PX4 leaves OFFBOARD.
        DeclareLaunchArgument('pilot_reaction', default_value='0.0'),
        # aD and MIS_TAKEOFF_ALT are both derived from zD — one number drives three things.
        DeclareLaunchArgument('zD', default_value='1.2'),
        # 1.5 > zD 1.2: take off high, descend onto target. Pass 'zD' to start at servo depth.
        DeclareLaunchArgument('takeoff_alt', default_value='1.5'),
        # Node default (0.5 m) straddles the depth stability threshold — tighten it here.
        DeclareLaunchArgument('takeoff_tolerance', default_value='0.10'),
        # 'launch' needed for observer experiments: fixed_eso converges in ~1 s post-gate.
        DeclareLaunchArgument('record_from', default_value='handover',
                              description='handover | launch'),
        # DIAGNOSTIC (E94). Feeds GROUND-TRUTH tilt into the loop: derotation (image_features),
        # setpoint (bridge corrects by EKF2 - truth), both, or replay (truth + a recorded error).
        # Never a scored or deployable configuration.
        DeclareLaunchArgument('attitude_oracle', default_value='none',
                              description='none | derotation | setpoint | both | replay'),
        # replay only: CSV of (t, roll_err, pitch_err) in radians, t = 0 at the handover.
        DeclareLaunchArgument('oracle_replay_csv', default_value=''),
    ]

    def takeoff_alt_of(context):
        """takeoff_alt, defaulting to zD. One resolver so PX4 and the gate cannot disagree."""
        raw = LaunchConfiguration('takeoff_alt').perform(context).strip()
        if not raw or raw.lower() == 'zd':
            return float(LaunchConfiguration('zD').perform(context))
        return float(raw)

    def _normalise_takeoff_alt(context):
        """Resolve takeoff_alt to a number before any consumer reads it.

        px4_takeoff_gate is a plain Node and re-implements the "empty means zD" rule as a
        PythonExpression, so there are two resolvers where the comment above promises one. Pinning
        the launch configuration here collapses them: downstream both see a plain number, and
        `takeoff_alt:=zD` works everywhere rather than reaching value_type=float as a string.
        """
        return [SetLaunchConfiguration('takeoff_alt', '%.6f' % takeoff_alt_of(context))]

    def eso_z_des_of(context):
        """eso_z_des, defaulting to zD - the same idiom as takeoff_alt_of, for the same reason."""
        raw = LaunchConfiguration('eso_z_des').perform(context).strip()
        return float(raw) if raw else float(LaunchConfiguration('zD').perform(context))

    def rotor_of(context):
        """(w_max, hover throttle, T/W) - one resolver, so PX4 and the bridge cannot disagree."""
        return rotor_setup(LaunchConfiguration('prop').perform(context),
                           int(LaunchConfiguration('battery_cells').perform(context)),
                           float(LaunchConfiguration('quad_mass').perform(context)))

    def _check_plant(context, *a, **k):
        """Refuse an unflyable plant or a stale hover_thrust before Gazebo starts, not after."""
        w_max, thr_hover, t_w = rotor_of(context)
        prop = LaunchConfiguration('prop').perform(context)
        cells = int(LaunchConfiguration('battery_cells').perform(context))
        mass = float(LaunchConfiguration('quad_mass').perform(context))
        # The bridge maps newtons through hover_thrust; one off MPC_THR_HOVER mis-scales every
        # command. An explicit value may only confirm the derivation.
        declared = LaunchConfiguration('hover_thrust').perform(context).strip()
        if declared and abs(float(declared) - thr_hover) > 0.005:
            raise RuntimeError(
                'hover_thrust:=%s but %s on %dS at %.2f kg derives %.4f. Omit hover_thrust to '
                'use the derived value.' % (declared, prop, cells, mass, thr_hover))
        return [LogInfo(msg='plant: %s on %dS at %.2f kg - rotor ceiling %.0f rad/s, '
                            'MPC_THR_HOVER %.4f, T/W %.2f'
                            % (prop, cells, mass, w_max, thr_hover, t_w))]

    def _indoor(context):
        """venue:=indoor or vicon - both are the GPS-denied lab, and every indoor decision holds
        for vicon too. One resolver, for the same reason takeoff_alt_of is one."""
        return LaunchConfiguration('venue').perform(context).strip().lower() in ('indoor', 'vicon')

    def _vicon(context):
        """venue:=vicon. Indoor, plus Vicon aiding EKF2 under aiding_policy."""
        return LaunchConfiguration('venue').perform(context).strip().lower() == 'vicon'

    def _full_pose(context):
        """venue:=vicon aiding_policy:=always: EKF2 fuses Vicon height and yaw too. Only safe when
        aiding is never cut, so EKF2_EV_CTRL and EKF2_HGT_REF come from this one resolver."""
        return (_vicon(context)
                and LaunchConfiguration('aiding_policy').perform(context).strip().lower() == 'always'
                and LaunchConfiguration('ev_velocity').perform(context).lower() != 'true')

    simulation = [
        include(SIM_PKG, 'gz_sim.launch.py',
                {'headless': LaunchConfiguration('headless'),
                 'world': WORLD,
                 'plant': 'px4',
                 'camera': LaunchConfiguration('camera'),
                 'target_scale': LaunchConfiguration('target_scale'),
                 'marker_dict': LaunchConfiguration('marker_dict'),
                 'camera_rate': LaunchConfiguration('camera_rate'),
                 'quad_mass': LaunchConfiguration('quad_mass')}),
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
                 'target_release': LaunchConfiguration('target_release'),
                 'target_profile': LaunchConfiguration('target_profile'),
                 'target_speed': LaunchConfiguration('target_speed'),
                 'target_yaw_rate': LaunchConfiguration('target_yaw_rate'),
                 'target_accel': LaunchConfiguration('target_accel'),
                 'target_heading': LaunchConfiguration('target_heading'),
                 'wind_velocity': LaunchConfiguration('wind_velocity'),
                 'blackout_at': LaunchConfiguration('blackout_at'),
                 'blackout_for': LaunchConfiguration('blackout_for'),
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
        cells = int(LaunchConfiguration('battery_cells').perform(context))
        w_max, thr_hover, _ = rotor_of(context)
        for i in (1, 2, 3, 4):
            env['PX4_PARAM_SIM_GZ_EC_MAX%d' % i] = '%d' % round(w_max)
        env['PX4_PARAM_MPC_THR_HOVER'] = '%.4f' % thr_hover
        # rcS defaults it to 4; PX4's simulated battery scales its pack voltage by it.
        env['PX4_PARAM_BAT1_N_CELLS'] = '%d' % cells

        # Loss of marker lock stops the setpoint stream. The default 0 = Position expects RC
        # that SITL has not, so the aircraft descends; 5 = Hold is the only recoverable mode.
        # Indoors Hold needs a horizontal estimate that does not exist, so 1 = Altitude.
        env['PX4_PARAM_COM_OBL_RC_ACT'] = '1' if _indoor(context) else '5'

        # BOTH branches, always: PX4 saves these into parameters.bson, so a one-sided value
        # would survive into the next run of the other venue.
        indoor = _indoor(context)
        # Bitmask: 7 is PX4's default, 0 is no GNSS aiding. EKF2 then dead-reckons
        # horizontally, which nothing downstream uses.
        env['PX4_PARAM_EKF2_GPS_CTRL'] = '0' if indoor else '7'
        # 0 barometric, 1 GPS, 3 vision. Indoors unaided the barometer is the ONLY height source.
        env['PX4_PARAM_EKF2_HGT_REF'] = ('3' if _full_pose(context) else '0') if indoor else '1'
        # See the imu_ctrl argument. Default 7 is PX4's own, so this is inert unless asked for.
        env['PX4_PARAM_EKF2_IMU_CTRL'] = LaunchConfiguration('imu_ctrl').perform(context).strip()
        # Vicon external vision. Bits: 0 horizontal position, 1 vertical position, 2 velocity,
        # 3 yaw. always: 11, the full pose - the bridge's frame check validates the datum against
        # the mag heading before the first sample, so fusing yaw can no longer hide a frame error.
        # Cut policies: 1, so the handover cut cannot remove the height reference at 1.2 m.
        # ev_velocity: 4. Set on BOTH branches for the parameters.bson reason above.
        _evvel = LaunchConfiguration('ev_velocity').perform(context).lower() == 'true'
        env['PX4_PARAM_EKF2_EV_CTRL'] = (
            ('4' if _evvel else '11' if _full_pose(context) else '1')
            if _vicon(context) else '0')
        # 1 = trust EKF2_EVP/EVA_NOISE rather than the message covariance; the bridge reports a
        # fixed variance, so there is nothing better in the message.
        env['PX4_PARAM_EKF2_EV_NOISE_MD'] = '1'
        env['PX4_PARAM_EKF2_EVP_NOISE'] = '0.02'
        env['PX4_PARAM_EKF2_EVA_NOISE'] = '0.05'
        # See the ev_delay argument; vicon_latency only shapes vicon_sim now.
        env['PX4_PARAM_EKF2_EV_DELAY'] = (
            '%.1f' % float(LaunchConfiguration('ev_delay').perform(context))
            if _vicon(context) else '0.0')

        # Set explicitly, not trusted to its default: PX4 intermittently refuses to arm with
        # "Preflight Fail: barometer 0 missing".
        env['PX4_PARAM_SIM_GZ_EN_BARO'] = '1'

        # 4 ignores every stick source; 1 is MAVLink only, so PX4 accepts sim_pilot's stream.
        # On the real aircraft this must be 0 (RC only) - the pilot is on a transmitter.
        env['PX4_PARAM_COM_RC_IN_MODE'] = '1' if indoor else '4'
        # Negative disables the stick override entirely (manual_control_params.yaml). In SITL the
        # "pilot" is a node, so nobody needs to grab the aircraft, and the override is pure hazard:
        # commander hands the aircraft back by setting the mode intent to POSCTL unconditionally,
        # which GPS-denied never returns. sim_pilot freezes its sticks as well; this is the backstop.
        # HARDWARE KEEPS THE DEFAULT 1.0 - there the override is a safety feature. Explicit on both
        # branches for the parameters.bson reason above.
        env['PX4_PARAM_MAN_OVERRIDE_SPD'] = '-1.0'
        # From boot until disarm, not the default "while armed": the start-up failures worth
        # diagnosing are exactly the runs that never arm, which would log nothing.
        env['PX4_PARAM_SDLOG_MODE'] = '1'

        # Takeoff has to deliver the aircraft to the depth image_features' aD was computed
        # for, or the feature vector is mis-scaled from the first frame. Same derivation, one
        # argument: the airframe's own default is only for a bare px4_sitl_default run.
        env['PX4_PARAM_MIS_TAKEOFF_ALT'] = '%.3f' % (takeoff_alt_of(context) + TAKEOFF_UNDERSHOOT)

        env['PX4_PARAM_EKF2_MAG_ACCLIM'] = '%.3f' % float(
            LaunchConfiguration('mag_acclim').perform(context))

        px4_proc = ExecuteProcess(cmd=[binary], cwd=rootfs, env=env,
                                  name='px4_sitl', output='screen')

        return [
            ExecuteProcess(cmd=[agent, 'udp4', '-p', '8888'],
                           name='micro_xrce_agent', output='log'),
            # px4-rc.gzsim already polls 30 s for the world; this just keeps the console clean.
            TimerAction(period=5.0, actions=[px4_proc]),
            # gz_bridge now refuses to boot on a sensor whose gz stream never arrived. Without
            # this the run would idle to px4_takeoff_gate's 60 s no-PX4 timeout instead.
            # Positive codes only: a negative one is the signal WE sent it during a teardown
            # already in progress, which is not a cause and must not be reported as one.
            RegisterEventHandler(OnProcessExit(
                target_action=px4_proc,
                on_exit=lambda event, context: (
                    [LogInfo(msg='px4_sitl exited %d during start-up - see its error above.'
                                 % event.returncode),
                     Shutdown(reason='px4 exited')]
                    if (event.returncode or 0) > 0 else []
                ))),
        ]

    def _state_adapter(context, *a, **k):
        # EV fusion resets EKF2 onto the Vicon (= gz world) frame, so adding the spawn origin
        # would count it twice; height is absolute only when EV height is fused too.
        vicon = _vicon(context)
        return [Node(
            package=PKG, executable='px4_state_adapter', name='px4_state_adapter',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'origin_north': 0.0 if vicon else SPAWN_NED[0],
                         'origin_east': 0.0 if vicon else SPAWN_NED[1],
                         'origin_down': 0.0 if _full_pose(context) else SPAWN_NED[2],
                         'frame_yaw_offset': FRAME_YAW_OFFSET}])]

    def _sim_pilot(context, *a, **k):
        """venue:=indoor only. Nothing else can get the aircraft off the ground there."""
        if not _indoor(context):
            return []
        return [Node(
            package=PKG, executable='sim_pilot', name='sim_pilot', output='screen',
            parameters=[{'use_sim_time': True,
                         # Same resolver as the bridge's, or the pilot stops climbing below the
                         # height the bridge is waiting for and the handover never happens.
                         'takeoff_altitude': takeoff_alt_of(context),
                         'hands_off_in_offboard': ParameterValue(
                             LaunchConfiguration('pilot_hands_off'), value_type=bool),
                         'pilot_reaction': ParameterValue(
                             LaunchConfiguration('pilot_reaction'), value_type=float)}])]

    def _vicon_nodes(context, *a, **k):
        """venue:=vicon only. vicon_sim stands in for the lab; the bridge is the flight article."""
        if not _vicon(context):
            return []
        policy = LaunchConfiguration('aiding_policy').perform(context)
        return [
            LogInfo(msg=('venue: vicon - EKF2 fuses the Vicon pose (position + yaw) all flight.'
                         if _full_pose(context) else
                         'venue: vicon - EKF2 aided by mocap, policy %s; aiding may be cut while '
                         'the loop has the aircraft.' % policy)),
            Node(package=SIM_PKG, executable='vicon_sim', name='vicon_sim', output='screen',
                 parameters=[{'use_sim_time': True,
                              'latency': ParameterValue(
                                  LaunchConfiguration('vicon_latency'), value_type=float),
                              'position_noise_sd': ParameterValue(
                                  LaunchConfiguration('vicon_noise_sd'), value_type=float),
                              'dropout_probability': ParameterValue(
                                  LaunchConfiguration('vicon_dropout_probability'),
                                  value_type=float)}]),
            Node(package=PKG, executable='vicon_px4_bridge', name='vicon_px4_bridge',
                 output='screen',
                 parameters=[{'use_sim_time': True,
                              'aiding_policy': LaunchConfiguration('aiding_policy'),
                              # vicon_sim publishes ENU with FLU body axes, as a Vicon volume
                              # calibrated Z-up does. The lab's own convention is confirmed by
                              # hand on the bench, not assumed from here.
                              'vicon_frame': 'enu',
                              # Empty unless ev_velocity:=true. /quad_state's twist is body FLU
                              # (gz_state_adapter.cpp:63); the bridge flips it to FRD.
                              'velocity_topic': PythonExpression(
                                  ["'/quad_state' if '",
                                   LaunchConfiguration('ev_velocity'), "'.lower()=='true' else ''"]),
                              'body_frame': 'flu',
                              # Feeds EKF2's own NED frame; the workspace datum is applied
                              # downstream by px4_state_adapter's frame_yaw_offset.
                              'yaw_offset': 0.0,
                              'stamp_source': 'header'}]),
        ]

    def _oracle(context):
        mode = LaunchConfiguration('attitude_oracle').perform(context).strip().lower()
        if mode not in ('none', 'derotation', 'setpoint', 'both', 'replay'):
            raise RuntimeError('attitude_oracle:=%s is not one of none, derotation, setpoint, '
                               'both, replay.' % mode)
        if mode == 'replay' and not LaunchConfiguration('oracle_replay_csv').perform(context):
            raise RuntimeError('attitude_oracle:=replay needs oracle_replay_csv.')
        return mode

    def _oracle_nodes(context, *a, **k):
        mode = _oracle(context)
        if mode == 'none':
            return []
        nodes = [
            LogInfo(msg='attitude_oracle:=%s - GROUND TRUTH is inside the loop. Diagnostic only.'
                        % mode),
            Node(package=PKG, executable='attitude_oracle', name='attitude_oracle',
                 output='screen',
                 parameters=[{'use_sim_time': True,
                              # A frame error is >= 90 deg; aided EKF2 at rest has read 1.6.
                              'max_tilt_disagreement': 5.0,
                              'replay_csv': (LaunchConfiguration('oracle_replay_csv')
                                             .perform(context) if mode == 'replay' else '')}])]
        if mode in ('derotation', 'both', 'replay'):
            # The unmodified tracking differentiator, fed the reference instead of EKF2, so the
            # derotation input differs from the flown one in its source and nothing else.
            nodes.append(Node(package=CTRL_PKG, executable='td_attitude',
                              name='td_attitude_reference', output='log',
                              parameters=[{'use_sim_time': True}],
                              remappings=[('quad_attitude', 'quad_attitude_reference'),
                                          ('attitude_estimates', 'attitude_estimates_reference'),
                                          ('attitude_td_error', 'attitude_td_error_reference')]))
        return nodes

    def _offboard_bridge(context, *a, **k):
        # Must come from the SAME resolver as MIS_TAKEOFF_ALT and the takeoff gate: left at the
        # node's own default the bridge never streams below it, and records loiter as servoing.
        return [Node(
            package=PKG, executable='px4_offboard_bridge', name='px4_offboard_bridge',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'hover_thrust': rotor_of(context)[1],
                         'mass': float(LaunchConfiguration('quad_mass').perform(context)),
                         'takeoff_altitude': takeoff_alt_of(context),
                         'bringup': ('pilot' if _indoor(context) else 'auto'),
                         # sim_pilot has no consent input and never will - it is not a person.
                         # hardware.launch.py is where this stays true.
                         'require_consent': False,
                         'offboard_recovery': ParameterValue(
                             LaunchConfiguration('offboard_recovery'), value_type=bool),
                         # Same value as the adapter's: one rotates into the workspace frame,
                         # the other rotates back out of it.
                         'frame_yaw_offset': FRAME_YAW_OFFSET,
                         'reference_attitude_correction':
                             _oracle(context) in ('setpoint', 'both', 'replay')}])]

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
                         LaunchConfiguration('takeoff_tolerance'), value_type=float),
                     # Indoors (vicon too) cs_gnss_pos never comes true. Yaw alignment is
                     # still required.
                     'require_gnss': ParameterValue(
                         PythonExpression(["'", LaunchConfiguration('venue'),
                                           "' == 'outdoor'"]), value_type=bool)}])

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
                                 LaunchConfiguration('gate_timeout'), value_type=float),
                             # tgt_position is the simulator's scenario generator; on hardware
                             # the target is a printed plate and no such topic exists.
                             'require_target': ParameterValue(
                                 PythonExpression(["'", LaunchConfiguration('venue'),
                                                   "' == 'outdoor'"]), value_type=bool)}])

    # pos_ctrl publishing desired_attitude is what tips px4_offboard_bridge into OFFBOARD,
    # so starting it here is the handover. att_ctrl stays off: PX4 owns the inner loop.
    control = RegisterEventHandler(OnProcessExit(
        target_action=gate,
        on_exit=lambda event, context: (
            ([include(CTRL_PKG, 'control.launch.py',
                      {'attitude_controller': 'false', 'zD': LaunchConfiguration('zD'),
                       'quad_mass': LaunchConfiguration('quad_mass'),
                       'velocity_source': LaunchConfiguration('velocity_source'),
                       'print_error': LaunchConfiguration('print_error')})]
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
                     'quad_mass': LaunchConfiguration('quad_mass'),
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
                     'detector_backend': LaunchConfiguration('detector_backend'),
                     'nano_error_correction': LaunchConfiguration('nano_error_correction'),
                     'nano_border_error_rate': LaunchConfiguration('nano_border_error_rate'),
                     'nano_box_filter': LaunchConfiguration('nano_box_filter'),
                     'nano_max_revisited': LaunchConfiguration('nano_max_revisited'),
                     'roi_margin': LaunchConfiguration('roi_margin'),
                     'use_aruco3_detection': LaunchConfiguration('use_aruco3_detection'),
                     'cv_num_threads': LaunchConfiguration('cv_num_threads'),
                     'min_marker_length_ratio': '%.6f' % presets.min_marker_ratio(
                         cam, float(LaunchConfiguration('target_scale').perform(context)),
                         float(LaunchConfiguration('zD').perform(context)),
                         float(LaunchConfiguration('aruco3_margin').perform(context))),
                     'attitude_topic': ('attitude_estimates_reference'
                                        if _oracle(context) in ('derotation', 'both', 'replay')
                                        else 'attitude_estimates'),
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
        args + [OpaqueFunction(function=_check_plant),
                OpaqueFunction(function=_normalise_takeoff_alt),
                OpaqueFunction(function=_bag_at_launch)] + simulation
        + [OpaqueFunction(function=_px4), OpaqueFunction(function=_state_adapter),
           # Before the pilot: EKF2 should be aided before anything tries to arm.
           OpaqueFunction(function=_vicon_nodes),
           OpaqueFunction(function=_sim_pilot),
           OpaqueFunction(function=_offboard_bridge), OpaqueFunction(function=_oracle_nodes),
           viz,
           estimation, takeoff_gate])
