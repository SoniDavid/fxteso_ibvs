"""The PX4-plant stack shared by sitl, hitl_world and hitl_flight: one set of resolvers, so the
modes cannot disagree on a derived value (MPC_THR_HOVER, MIS_TAKEOFF_ALT, the EKF2 set).

  sitl         gz world + PX4 SITL + the whole stack
  hitl_world   gz world, scenario, vicon_sim and hitl_bridge (USB to the real FC)
  hitl_flight  the companion's side: uXRCE agent on TELEM2, px4_state_adapter down
"""
import importlib.util
import json
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

MODES = ('sitl', 'hitl_world', 'hitl_flight')

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
# ROMFS/px4fmu_common/init.d/airframes/1010_f450_quad_x.hil: the same F450 on the real FC.
HITL_AUTOSTART = 1010

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
    Throttle maps linearly onto rotor velocity while thrust goes as omega^2."""
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


# Spawn pose from worlds/ibvs.sdf in NED, directly over the target; px4_state_adapter adds it back.
SPAWN_NED = (-10.0, -10.0, -0.1)

# PX4's takeoff settles this far below its setpoint; added to MIS_TAKEOFF_ALT only, so
# takeoff_alt is the altitude actually reached.
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


def build(mode='sitl'):
    if mode not in MODES:
        raise RuntimeError('px4_stack mode %r is not one of %s.' % (mode, ', '.join(MODES)))
    hitl = mode != 'sitl'
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
        # Velocity in pos_ctrl's Coriolis term; off keeps the Vicon-aided estimate out of the loop.
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
        DeclareLaunchArgument('eso_z_des', default_value=''),
        # Known harmful: re-arms OFFBOARD off the heartbeat while the loop is still blind.
        DeclareLaunchArgument('offboard_recovery', default_value='false'),
        # |qpsi| at handover separates held from collapsed runs at ~0.17; 0 disables.
        DeclareLaunchArgument('max_feature_error', default_value='0.15'),
        # EKF2 pitch bias ~0.019 rad for first 10 s → 0.18 m/s² uncommanded acceleration.
        DeclareLaunchArgument('estimators_ready', default_value='5.0'),
        # PX4 yaw unwinds over ~100 s after takeoff; target is held so waiting costs only wall clock.
        DeclareLaunchArgument('gate_timeout', default_value='120.0'),
        # vicon: indoor plus EKF2 aided by mocap (the flight configuration); indoor: unaided.
        DeclareLaunchArgument('venue', default_value='vicon',
                              choices=['outdoor', 'indoor', 'vicon']),
        # venue:=vicon only. vicon_sim's imitation of the lab; the bridge is the one that flies.
        DeclareLaunchArgument('vicon_latency', default_value='0.02'),
        DeclareLaunchArgument('vicon_noise_sd', default_value='0.001'),
        DeclareLaunchArgument('vicon_dropout_probability', default_value='0.0'),
        # EKF2_EV_DELAY, ms. 0: the bridge's timestamp_sample already carries the capture time, so
        # EKF2 subtracting vicon_latency again would date every sample that much too early.
        DeclareLaunchArgument('ev_delay', default_value='0.0'),
        # always: full pose into EKF2 all flight (the flight configuration); gated: horizontal
        # position only, cut at the handover.
        DeclareLaunchArgument('aiding_policy', default_value='always',
                              choices=['gated', 'always', 'oneshot', 'manual']),
        # DIAGNOSTIC, venue:=vicon only: aid EKF2 with velocity alone, as optical flow would.
        DeclareLaunchArgument('ev_velocity', default_value='false',
                              choices=['true', 'false']),
        # EKF2_IMU_CTRL bits: 0 gyro bias, 1 accel bias, 2 gravity. 7 is PX4's default; 5 stops the
        # unaided tilt/accel-bias trade.
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
        # Degrees from +x. 90 = the camera's wide axis (~2x the budget), as the lab cart runs.
        DeclareLaunchArgument('target_heading', default_value='90.0'),
        # settled: the target moves once the loop holds it centred; control: on the loop's first
        # command, ~1.8 s before PX4 grants OFFBOARD.
        DeclareLaunchArgument('target_release', default_value='settled',
                              description='control | settled'),
        # settled only: feature offset the loop must hold before release.
        DeclareLaunchArgument('target_settle_offset', default_value='0.05'),
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
        # gate covers the handover (the recorder takes ~4 s to subscribe); launch also the observer.
        DeclareLaunchArgument('record_from', default_value='handover',
                              description='handover | gate | launch'),
        # DIAGNOSTIC: ground-truth tilt into derotation, the setpoint, both, or replay. Never scored.
        DeclareLaunchArgument('attitude_oracle', default_value='none',
                              description='none | derotation | setpoint | both | replay'),
        # replay only: CSV of (t, roll_err, pitch_err) in radians, t = 0 at the handover.
        DeclareLaunchArgument('oracle_replay_csv', default_value=''),
    ]
    if hitl:
        # Both HITL terminals declare these; pass any override to BOTH, as with every argument.
        args += [
            # sim: sim_pilot flies the takeoff over MAVLink sticks. rc: the real transmitter does,
            # and the handover needs consent - the hardware procedure, rehearsed.
            DeclareLaunchArgument('pilot', default_value='sim', choices=['sim', 'rc']),
            # pilot:=rc only. RC AUX channel for consent (needs RC_MAP_AUXn); 0 = ~/handover service.
            DeclareLaunchArgument('consent_rc_aux', default_value='0'),
            # USB MAVLink to the FC, by-id so it survives the reboot after a param change.
            # none runs hitl_bridge without an FC: gz side and RTF only.
            DeclareLaunchArgument(
                'fc_device',
                default_value='/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00'),
            # Real time factor below which hitl_bridge withholds ready: HITL is not lockstep.
            DeclareLaunchArgument('min_rtf', default_value='0.95'),
            # QGC reaches the FC through hitl_bridge while it holds USB. 0 disables.
            DeclareLaunchArgument('gcs_udp_port', default_value='14550'),
            # false: you run MicroXRCEAgent yourself; two agents cannot share the TELEM2 tty.
            DeclareLaunchArgument('agent', default_value='true'),
            # The USB-UART on TELEM2 (UXRCE_DDS_CFG 102).
            DeclareLaunchArgument('agent_device', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('agent_baud', default_value='921600'),
        ]

    def takeoff_alt_of(context):
        """takeoff_alt, defaulting to zD. One resolver so PX4 and the gate cannot disagree."""
        raw = LaunchConfiguration('takeoff_alt').perform(context).strip()
        if not raw or raw.lower() == 'zd':
            return float(LaunchConfiguration('zD').perform(context))
        return float(raw)

    def _normalise_takeoff_alt(context):
        """Pin takeoff_alt to a number, so the takeoff gate's PythonExpression sees one too."""
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
                 'target_settle_offset': LaunchConfiguration('target_settle_offset'),
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

    def _pilot_rc(context):
        """pilot:=rc - a person on a transmitter. HITL only; SITL's pilot is always sim_pilot."""
        return hitl and LaunchConfiguration('pilot').perform(context).strip().lower() == 'rc'

    def px4_params(context):
        """Every PX4 parameter this launch decides, by name. SITL hands them to PX4 as
        PX4_PARAM_* environment variables, HITL pushes them to the real FC through hitl_bridge."""
        p = {}
        # Rotor ceiling and hover anchor are one derivation; PX4 must not carry a second copy.
        cells = int(LaunchConfiguration('battery_cells').perform(context))
        w_max, thr_hover, _ = rotor_of(context)
        p['MPC_THR_HOVER'] = '%.4f' % thr_hover
        # rcS defaults it to 4; PX4's simulated battery scales its pack voltage by it.
        p['BAT1_N_CELLS'] = '%d' % cells

        # Offboard loss: 5 = Hold outdoors (0 = Position descends without RC); indoors 1 = Altitude.
        p['COM_OBL_RC_ACT'] = '1' if _indoor(context) else '5'

        # BOTH branches, always: PX4 saves these into parameters.bson, so a one-sided value
        # would survive into the next run of the other venue.
        indoor = _indoor(context)
        # Bitmask: 7 is PX4's default, 0 is no GNSS aiding. EKF2 then dead-reckons
        # horizontally, which nothing downstream uses.
        p['EKF2_GPS_CTRL'] = '0' if indoor else '7'
        # 0 barometric, 1 GPS, 3 vision. Indoors unaided the barometer is the ONLY height source.
        p['EKF2_HGT_REF'] = ('3' if _full_pose(context) else '0') if indoor else '1'
        # See the imu_ctrl argument. Default 7 is PX4's own, so this is inert unless asked for.
        p['EKF2_IMU_CTRL'] = LaunchConfiguration('imu_ctrl').perform(context).strip()
        # EV bits: 0 horiz pos, 1 vert pos, 2 vel, 3 yaw. always 11 (full pose), cut policies 1
        # (keep the height reference), ev_velocity 4.
        _evvel = LaunchConfiguration('ev_velocity').perform(context).lower() == 'true'
        p['EKF2_EV_CTRL'] = (
            ('4' if _evvel else '11' if _full_pose(context) else '1')
            if _vicon(context) else '0')
        # 1 = trust EKF2_EVP/EVA_NOISE rather than the message covariance; the bridge reports a
        # fixed variance, so there is nothing better in the message.
        p['EKF2_EV_NOISE_MD'] = '1'
        p['EKF2_EVP_NOISE'] = '0.02'
        p['EKF2_EVA_NOISE'] = '0.05'
        # See the ev_delay argument; vicon_latency only shapes vicon_sim now.
        p['EKF2_EV_DELAY'] = (
            '%.1f' % float(LaunchConfiguration('ev_delay').perform(context))
            if _vicon(context) else '0.0')

        # 4 ignores every stick source; 1 is MAVLink only, so PX4 accepts sim_pilot's stream.
        # On the real aircraft this must be 0 (RC only) - the pilot is on a transmitter.
        p['COM_RC_IN_MODE'] = '0' if _pilot_rc(context) else '1' if indoor else '4'
        # Negative disables the stick override, which GPS-denied would turn into a POSCTL lockout.
        # Hardware and pilot:=rc keep the default 1.0: there it is a safety feature.
        p['MAN_OVERRIDE_SPD'] = '1.0' if _pilot_rc(context) else '-1.0'
        # From boot until disarm, not the default "while armed": the start-up failures worth
        # diagnosing are exactly the runs that never arm, which would log nothing.
        p['SDLOG_MODE'] = '1'

        # Takeoff must reach the depth image_features' aD assumes.
        p['MIS_TAKEOFF_ALT'] = '%.3f' % (takeoff_alt_of(context) + TAKEOFF_UNDERSHOOT)

        p['EKF2_MAG_ACCLIM'] = '%.3f' % float(
            LaunchConfiguration('mag_acclim').perform(context))

        if hitl:
            # SITL's airframe disables both loss failsafes (no RC, no GCS); the FC keeps whatever
            # was saved last, so HITL states them. rc: hardware.params' values.
            p['NAV_RCL_ACT'] = '3' if _pilot_rc(context) else '0'
            p['NAV_DLL_ACT'] = '0'
            # SITL's posix rcS values, so HITL matches it: EKF2's IMU rate (FC default 200) and the
            # log profile with full-rate estimator topics (FC default 1 logs EV aid at ~2 Hz).
            p['IMU_INTEG_RATE'] = '250'
            p['SDLOG_PROFILE'] = '131'
        return p

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

        # Keep model.sdf's maxRotVelocity above these or the motor model clamps silently.
        w_max = rotor_of(context)[0]
        for i in (1, 2, 3, 4):
            env['PX4_PARAM_SIM_GZ_EC_MAX%d' % i] = '%d' % round(w_max)
        env.update({'PX4_PARAM_' + name: value for name, value in px4_params(context).items()})

        # Set explicitly, not trusted to its default: PX4 intermittently refuses to arm with
        # "Preflight Fail: barometer 0 missing".
        env['PX4_PARAM_SIM_GZ_EN_BARO'] = '1'

        px4_proc = ExecuteProcess(cmd=[binary], cwd=rootfs, env=env,
                                  name='px4_sitl', output='screen')

        return [
            ExecuteProcess(cmd=[agent, 'udp4', '-p', '8888'],
                           name='micro_xrce_agent', output='log'),
            # px4-rc.gzsim already polls 30 s for the world; this just keeps the console clean.
            TimerAction(period=5.0, actions=[px4_proc]),
            # End the run if px4 dies in start-up; negative codes are our own teardown signal.
            RegisterEventHandler(OnProcessExit(
                target_action=px4_proc,
                on_exit=lambda event, context: (
                    [LogInfo(msg='px4_sitl exited %d during start-up - see its error above.'
                                 % event.returncode),
                     Shutdown(reason='px4 exited')]
                    if (event.returncode or 0) > 0 else []
                ))),
        ]

    def _hitl_agent(context, *a, **k):
        """hitl: the uXRCE-DDS agent on the TELEM2 USB-UART, where SITL uses UDP."""
        if LaunchConfiguration('agent').perform(context).lower() != 'true':
            return [LogInfo(msg='agent:=false - expecting your own MicroXRCEAgent on TELEM2.')]
        agent = LaunchConfiguration('xrce_agent').perform(context)
        if not os.path.isfile(agent):
            raise RuntimeError(
                '%s not found. Pass xrce_agent:=<path to MicroXRCEAgent>.' % agent)
        device = LaunchConfiguration('agent_device').perform(context)
        if not os.path.exists(device):
            raise RuntimeError('%s not found - is the TELEM2 USB-UART plugged in? Pass '
                               'agent_device:=<tty>.' % device)
        return [ExecuteProcess(
            cmd=[agent, 'serial', '--dev', device,
                 '-b', LaunchConfiguration('agent_baud').perform(context)],
            name='micro_xrce_agent', output='log')]

    def _hitl_bridge(context, *a, **k):
        """hitl_world: gz sensors to the FC as HIL_SENSOR, its HIL_ACTUATOR_CONTROLS to the rotors,
        and the px4_params() set pushed and verified on the FC before anything flies."""
        return [Node(
            package=SIM_PKG, executable='hitl_bridge.py', name='hitl_bridge', output='screen',
            parameters=[{'fc_device': LaunchConfiguration('fc_device').perform(context),
                         'world': WORLD, 'model': MODEL,
                         'px4_params': json.dumps(px4_params(context), sort_keys=True),
                         'autostart': HITL_AUTOSTART,
                         # Same linear ESC map as SIM_GZ_EC_MIN/MAX, so MPC_THR_HOVER holds.
                         'rotor_min': ROTOR_MIN,
                         'rotor_max': float(round(rotor_of(context)[0])),
                         'min_rtf': ParameterValue(LaunchConfiguration('min_rtf'),
                                                   value_type=float),
                         'gcs_udp_port': ParameterValue(LaunchConfiguration('gcs_udp_port'),
                                                        value_type=int)}])]

    def _hitl_wait(context, *a, **k):
        """hitl: exits 0 once hitl_bridge is ready AND pushed the params this terminal derives -
        two terminals given different arguments would otherwise fly two different aircraft."""
        return [Node(
            package=PKG, executable='hitl_wait.py', name='hitl_wait', output='screen',
            parameters=[{'px4_params': json.dumps(px4_params(context), sort_keys=True)}])]

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
        if not _indoor(context) or _pilot_rc(context):
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
        """venue:=vicon only: vicon_sim stands in for the lab, the bridge is the flight article."""
        if not _vicon(context):
            return []
        policy = LaunchConfiguration('aiding_policy').perform(context)
        nodes = [
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
                              # ENU with FLU body axes, as a Z-up Vicon volume; confirm in the lab.
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
        return {'sitl': nodes, 'hitl_world': nodes[:2], 'hitl_flight': nodes[2:]}[mode]

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
        params = {}
        if hitl:
            params = {
                # A person on a transmitter gives consent, as on the aircraft.
                'bringup': 'pilot' if _pilot_rc(context) or _indoor(context) else 'auto',
                'require_consent': _pilot_rc(context),
                'consent_rc_aux': int(LaunchConfiguration('consent_rc_aux').perform(context)),
                # The FC's clock is not gz's: stamp in the agent's wall-clock domain.
                'stamp_clock': 'system'}
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
                             _oracle(context) in ('setpoint', 'both', 'replay')} | params])]

    viz = include(UTILS_PKG, 'viz.launch.py', {'foxglove': LaunchConfiguration('foxglove'),
                                               'camera': LaunchConfiguration('camera')})

    def _bag_at_launch(context, *a, **k):
        """record_from:=launch: record from the start, for the observer runs."""
        if LaunchConfiguration('record_from').perform(context).lower() != 'launch':
            return []
        return _bag(context)

    def _bag(context, *a, **k):
        if LaunchConfiguration('rosbag').perform(context).lower() != 'true':
            return []
        # mcap, not rosbag2's default sqlite3: Foxglove Studio cannot open .db3.
        out = os.path.join('bags', ('hitl_' if hitl else 'px4_') + time.strftime('%Y%m%d_%H%M%S'))
        return [ExecuteProcess(
            # Most recorded topics have no header stamp, so the recorder's clock is the time axis.
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
               if LaunchConfiguration('record_from').perform(context).lower() == 'handover'
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
            + ([OpaqueFunction(function=_bag)]
               if LaunchConfiguration('record_from').perform(context).lower() == 'gate' else [])
            if event.returncode == 0 else
            # EKF2 never recovers from a failed initialisation: end the run now.
            [LogInfo(msg='px4_takeoff_gate failed - estimators not started. See its error '
                         'above.'), Shutdown(reason='takeoff gate failed')]
        )))

    head = [OpaqueFunction(function=_check_plant), OpaqueFunction(function=_normalise_takeoff_alt)]

    if mode == 'hitl_world':
        # No PX4 process and no agent: PX4 is the FC on USB, and its DDS side is hitl_flight's.
        return LaunchDescription(
            args + head + simulation
            + [OpaqueFunction(function=_vicon_nodes), OpaqueFunction(function=_hitl_bridge)])

    if mode == 'hitl_flight':
        # Nothing arms until hitl_wait passes; it is an OpaqueFunction, so matched by executable.
        armed = RegisterEventHandler(OnProcessExit(
            target_action=lambda action: getattr(action, 'node_executable', None)
            == 'hitl_wait.py',
            on_exit=lambda event, context: (
                [OpaqueFunction(function=_sim_pilot), OpaqueFunction(function=_offboard_bridge),
                 takeoff_gate]
                if event.returncode == 0 else
                [LogInfo(msg='hitl_wait failed - nothing armed. See its error above.'),
                 Shutdown(reason='hitl_wait failed')])))
        return LaunchDescription(
            args + head + [OpaqueFunction(function=_bag_at_launch),
                           OpaqueFunction(function=_hitl_agent),
                           OpaqueFunction(function=_state_adapter),
                           OpaqueFunction(function=_vicon_nodes),
                           OpaqueFunction(function=_oracle_nodes),
                           armed, OpaqueFunction(function=_hitl_wait), viz,
                           estimation])

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
