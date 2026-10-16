#!/usr/bin/env python3
"""Gazebo Harmonic <-> a real PX4 FC in HITL over USB MAVLink (Classic's mavlink_interface for gz):
gz sensors -> HIL_SENSOR/HIL_GPS, HIL_ACTUATOR_CONTROLS -> rotors, QGC forwarded on UDP.
Conversions mirror the fork's GZBridge.cpp. Ready (latched ~/status) once the params are pushed
and verified and the RTF holds: HITL is not lockstep.
"""
import json
import math
import os
import socket
import struct
import threading
import time

import rclpy
import serial
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.magnetometer_pb2 import Magnetometer
from gz.msgs10.navsat_pb2 import NavSat
from gz.transport13 import Node as GzNode
from pymavlink.dialects.v20 import common as mavlink
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# HIL_SENSOR fields_updated, as PX4's mavlink_receiver groups them (SensorSource).
ACCEL, GYRO, MAG, BARO = 0b111, 0b111000, 0b111000000, 0b1101000000000
INT_TYPES = (mavlink.MAV_PARAM_TYPE_UINT8, mavlink.MAV_PARAM_TYPE_INT8,
             mavlink.MAV_PARAM_TYPE_UINT16, mavlink.MAV_PARAM_TYPE_INT16,
             mavlink.MAV_PARAM_TYPE_UINT32, mavlink.MAV_PARAM_TYPE_INT32)
TEMPERATURE_C = 15.0     # GZBridge's _temperature with no airspeed sensor on the model
GPS_SATELLITES = 10      # SIM_GPS_USED default
ACTUATOR_TIMEOUT = 0.1   # s without HIL_ACTUATOR_CONTROLS before the rotors are stopped
HIL_FLOW_TIMEOUT = 0.5   # s without HIL_ACTUATOR_CONTROLS before the FC counts as not in HIL
RTF_HOLD = 5.0           # s the RTF must stay above min_rtf before ready


def float_bits(value, ptype):
    """PX4 encodes integer params bytewise in PARAM_VALUE's float field."""
    if ptype in INT_TYPES:
        return struct.unpack('<f', struct.pack('<i', int(round(float(value)))))[0]
    return float(value)


def decode(param_value, ptype):
    if ptype in INT_TYPES:
        return struct.unpack('<i', struct.pack('<f', param_value))[0]
    return param_value


def same(want, have, ptype):
    if ptype in INT_TYPES:
        return int(round(float(want))) == have
    # REAL32 on the FC: compare at float32 precision.
    return struct.pack('<f', float(want)) == struct.pack('<f', have)


class Link:
    """The USB serial link: one writer lock, a reader thread, and reconnect after a reboot."""

    def __init__(self, device, on_message, on_raw, log):
        self.device = device
        self.on_message = on_message
        self.on_raw = on_raw
        self.log = log
        self.lock = threading.Lock()
        # pymavlink's encoder (sequence number, buffer) is not thread-safe; four threads send.
        self.send_lock = threading.Lock()
        self.ser = None
        self.mav = mavlink.MAVLink(self, srcSystem=1, srcComponent=200)
        self.mav.robust_parsing = True
        self.connected = threading.Event()
        self.stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def write(self, buf):
        """pymavlink's file interface; drops the write while disconnected."""
        with self.lock:
            if self.ser is None:
                return
            try:
                self.ser.write(buf)
            except (serial.SerialException, OSError):
                self._drop()

    def send(self, msg):
        if self.ser is None:
            return
        with self.send_lock:
            self.mav.send(msg)

    def _drop(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected.clear()

    def disconnect(self):
        with self.lock:
            self._drop()

    def _reader(self):
        parser = mavlink.MAVLink(None)
        parser.robust_parsing = True
        while not self.stop:
            if self.ser is None:
                if not os.path.exists(self.device):
                    time.sleep(0.2)
                    continue
                try:
                    ser = serial.Serial(self.device, 921600, timeout=0.02)
                except (serial.SerialException, OSError):
                    time.sleep(0.5)
                    continue
                with self.lock:
                    self.ser = ser
                self.connected.set()
                self.log('connected to %s' % self.device)
            try:
                data = self.ser.read(4096)
            except (serial.SerialException, OSError, TypeError, AttributeError):
                self.log('lost %s' % self.device)
                self.disconnect()
                continue
            if not data:
                continue
            self.on_raw(data)
            try:
                msgs = parser.parse_buffer(data) or []
            except mavlink.MAVError:
                continue
            for msg in msgs:
                self.on_message(msg)


class HitlBridge(Node):

    def __init__(self):
        super().__init__('hitl_bridge')
        p = self.declare_parameter
        self.device = p('fc_device', 'none').value
        if self.device.lower() in ('', 'none'):
            self.device = ''
        world = p('world', 'ibvs').value
        model = p('model', 'F450').value
        self.px4_params = json.loads(p('px4_params', '{}').value)
        self.autostart = p('autostart', 1010).value
        self.rotor_min = p('rotor_min', 150.0).value
        self.rotor_max = p('rotor_max', 1089.0).value
        self.min_rtf = p('min_rtf', 0.95).value
        gcs_port = p('gcs_udp_port', 14550).value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', qos)

        self.state_lock = threading.Lock()
        self.mag = None            # latest (x, y, z) gauss FRD, sent once
        self.baro = None           # latest pressure Pa, sent once
        self.fc_sysid = None
        self.last_actuators = 0.0  # monotonic s
        self.armed = False
        self.hil_on = False
        self.params_ok = False
        self.params_reason = 'no FC' if not self.device else 'waiting for the FC heartbeat'
        self.param_values = {}     # name -> (value, type), from PARAM_VALUE
        self.param_event = threading.Condition()
        self.rtf = float('nan')
        self.rtf_ok_since = None
        self.rtf_window = []       # (wall, sim) pairs
        self.motors_zeroed = True

        self.gcs = None
        if gcs_port > 0:
            self.gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.gcs.bind(('0.0.0.0', 0))
            self.gcs.setblocking(False)
            self.gcs_addr = ('127.0.0.1', gcs_port)
            threading.Thread(target=self._gcs_reader, daemon=True).start()

        self.link = None
        if self.device:
            self.link = Link(self.device, self._on_mavlink, self._to_gcs,
                             lambda m: self.get_logger().info(m))

        # gz side: the topics the fork's GZBridge reads, and the one its ESC interface writes.
        sensor = '/world/%s/model/%s/link/base_link/sensor/' % (world, model)
        self.gz = GzNode()
        self.motor_pub = self.gz.advertise('/%s/command/motor_speed' % model, Actuators)
        for msg_type, topic, cb in (
                (IMU, sensor + 'imu_sensor/imu', self._on_imu),
                (Magnetometer, sensor + 'magnetometer_sensor/magnetometer', self._on_mag),
                (FluidPressure, sensor + 'air_pressure_sensor/air_pressure', self._on_baro),
                (NavSat, sensor + 'navsat_sensor/navsat', self._on_navsat)):
            if not self.gz.subscribe(msg_type, topic, cb):
                raise RuntimeError('cannot subscribe to gz topic %s' % topic)

        self.create_timer(1.0, self._heartbeat_and_status, clock=rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.create_timer(0.02, self._actuator_watchdog, clock=rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.STEADY_TIME))
        if self.link is not None:
            threading.Thread(target=self._param_sync, daemon=True).start()
        self._publish_status()

    # --- gz -> FC ----------------------------------------------------------------------------

    def _on_imu(self, msg):
        sim = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        self._track_rtf(sim)
        if self.link is None:
            return
        # FLU -> FRD: GZBridge rotates by q(0, 1, 0, 0), i.e. (x, -y, -z).
        a, w = msg.linear_acceleration, msg.angular_velocity
        with self.state_lock:
            mag, self.mag = self.mag, None
            baro, self.baro = self.baro, None
        fields = ACCEL | GYRO
        xmag = ymag = zmag = 0.0
        if mag is not None:
            fields |= MAG
            xmag, ymag, zmag = mag
        abs_hpa = alt = 0.0
        if baro is not None:
            fields |= BARO
            abs_hpa = baro / 100.0
            alt = 44330.0 * (1.0 - (baro / 101325.0) ** (1.0 / 5.255))
        self.link.send(mavlink.MAVLink_hil_sensor_message(
            int(time.time() * 1e6), a.x, -a.y, -a.z, w.x, -w.y, -w.z, xmag, ymag, zmag,
            abs_hpa, 0.0, alt, TEMPERATURE_C, fields, 0))

    def _on_mag(self, msg):
        # GZBridge: gz publishes gauss in a left-handed frame (gz-sim#2460) -> (-y, -x, z).
        f = msg.field_tesla
        with self.state_lock:
            self.mag = (-f.y, -f.x, f.z)

    def _on_baro(self, msg):
        with self.state_lock:
            self.baro = msg.pressure

    def _on_navsat(self, msg):
        # GZBridge's 3D-fix figures. Its F9P noise model is not reproduced: GPS is unfused indoors.
        if self.link is None:
            return
        vn, ve, vd = msg.velocity_north, msg.velocity_east, -msg.velocity_up
        cog = math.degrees(math.atan2(ve, vn)) % 360.0
        self.link.send(mavlink.MAVLink_hil_gps_message(
            int(time.time() * 1e6), 3, int(msg.latitude_deg * 1e7), int(msg.longitude_deg * 1e7),
            int(msg.altitude * 1000), 90, 178, int(math.hypot(vn, ve) * 100),
            int(vn * 100), int(ve * 100), int(vd * 100), int(cog * 100), GPS_SATELLITES))

    def _track_rtf(self, sim):
        wall = time.monotonic()
        win = self.rtf_window
        win.append((wall, sim))
        while win and wall - win[0][0] > 2.0:
            win.pop(0)
        if len(win) > 10 and win[-1][0] - win[0][0] > 1.0:
            self.rtf = (win[-1][1] - win[0][1]) / (win[-1][0] - win[0][0])
            if self.rtf >= self.min_rtf:
                if self.rtf_ok_since is None:
                    self.rtf_ok_since = wall
            else:
                self.rtf_ok_since = None

    # --- FC -> gz ----------------------------------------------------------------------------

    def _on_mavlink(self, msg):
        mid = msg.get_msgId()
        if mid == mavlink.MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            self.armed = bool(msg.mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.hil_on = bool(msg.mode & mavlink.MAV_MODE_FLAG_HIL_ENABLED)
            self.last_actuators = time.monotonic()
            # pwm_out_sim scales motors to [0, 1]; SIM_GZ_EC_MIN/MAX's linear map onto rad/s.
            u = msg.controls[:4]
            self._motors([self.rotor_min + max(0.0, min(1.0, c)) * (self.rotor_max - self.rotor_min)
                          for c in u] if self.armed else [0.0] * 4)
        elif mid == mavlink.MAVLINK_MSG_ID_HEARTBEAT:
            if msg.get_srcComponent() == mavlink.MAV_COMP_ID_AUTOPILOT1 and self.fc_sysid is None:
                self.fc_sysid = msg.get_srcSystem()
        elif mid == mavlink.MAVLINK_MSG_ID_PARAM_VALUE:
            name = msg.param_id.rstrip('\x00') if isinstance(msg.param_id, str) \
                else msg.param_id.decode().rstrip('\x00')
            with self.param_event:
                self.param_values[name] = (decode(msg.param_value, msg.param_type),
                                           msg.param_type)
                self.param_event.notify_all()

    def _motors(self, velocities):
        out = Actuators()
        out.velocity.extend(velocities)
        self.motor_pub.publish(out)
        self.motors_zeroed = not any(velocities)

    def _actuator_watchdog(self):
        # A silent FC is a dead FC: stop the rotors rather than hold the last command.
        if (time.monotonic() - self.last_actuators > ACTUATOR_TIMEOUT
                and not self.motors_zeroed):
            self.get_logger().warn('no HIL_ACTUATOR_CONTROLS for %.0f ms - rotors stopped'
                                   % (ACTUATOR_TIMEOUT * 1e3))
            self._motors([0.0] * 4)

    # --- QGC ---------------------------------------------------------------------------------

    def _to_gcs(self, data):
        if self.gcs is not None:
            try:
                self.gcs.sendto(data, self.gcs_addr)
            except OSError:
                pass

    def _gcs_reader(self):
        while True:
            try:
                data, _ = self.gcs.recvfrom(4096)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            except OSError:
                time.sleep(0.1)
                continue
            if self.link is not None:
                self.link.write(data)

    # --- parameters --------------------------------------------------------------------------

    def _read(self, name, timeout=1.0, tries=3):
        for _ in range(tries):
            with self.param_event:
                self.param_values.pop(name, None)
            self.link.send(mavlink.MAVLink_param_request_read_message(
                self.fc_sysid, 1, name.encode(), -1))
            with self.param_event:
                if self.param_event.wait_for(lambda: name in self.param_values, timeout):
                    return self.param_values[name]
        return None

    def _write(self, name, value, ptype):
        with self.param_event:
            self.param_values.pop(name, None)
        self.link.send(mavlink.MAVLink_param_set_message(
            self.fc_sysid, 1, name.encode(), float_bits(value, ptype), ptype))
        with self.param_event:
            if self.param_event.wait_for(lambda: name in self.param_values, 1.5):
                return self.param_values[name]
        return None

    def _fail(self, reason):
        self.params_reason = reason
        self.get_logger().error(reason)
        self._publish_status()

    def _param_sync(self):
        log = self.get_logger()
        for attempt in (1, 2):
            self.link.connected.wait()
            deadline = time.monotonic() + 20.0
            while self.fc_sysid is None and time.monotonic() < deadline:
                time.sleep(0.1)
            if self.fc_sysid is None:
                return self._fail('no PX4 heartbeat on %s' % self.device)
            self.params_reason = 'checking FC parameters'
            for name, want in (('SYS_AUTOSTART', self.autostart), ('SYS_HITL', 1)):
                got = self._read(name)
                if got is None or got[0] != want:
                    return self._fail(
                        '%s is %s, need %s. Flash the fork, select airframe %d (HIL F450) and '
                        'reboot - see RUNNING_HITL.md.'
                        % (name, None if got is None else got[0], want, self.autostart))
            changed = []
            for name in sorted(self.px4_params):
                want = self.px4_params[name]
                got = self._read(name)
                if got is None:
                    return self._fail('FC has no parameter %s - wrong firmware?' % name)
                have, ptype = got
                if same(want, have, ptype):
                    continue
                echo = self._write(name, want, ptype)
                if echo is None or not same(want, echo[0], ptype):
                    return self._fail('FC refused %s := %s (still %s)'
                                      % (name, want, None if echo is None else echo[0]))
                changed.append('%s %s -> %s' % (name, have, want))
            if not changed:
                log.info('FC parameters verified: %d of %d match the launch'
                         % (len(self.px4_params), len(self.px4_params)))
                self.params_ok = True
                self.params_reason = 'parameters verified'
                self._publish_status()
                return
            for line in changed:
                log.info('set ' + line)
            if attempt == 2:
                return self._fail('parameters changed again after a reboot: %s' % changed)
            if self.armed:
                return self._fail('parameters changed while armed - not rebooting the FC')
            # Let PX4's param autosave land before the reboot.
            self.params_reason = 'rebooting the FC to apply %d parameters' % len(changed)
            self._publish_status()
            log.info(self.params_reason)
            time.sleep(2.0)
            self.link.send(mavlink.MAVLink_command_long_message(
                self.fc_sysid, 1, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                1, 0, 0, 0, 0, 0, 0))
            time.sleep(1.0)
            self.link.disconnect()
            self.fc_sysid = None
            time.sleep(3.0)

    # --- status ------------------------------------------------------------------------------

    def _ready(self):
        if not self.params_ok:
            return False, self.params_reason
        if time.monotonic() - self.last_actuators > HIL_FLOW_TIMEOUT or not self.hil_on:
            return False, 'no HIL_ACTUATOR_CONTROLS from the FC - is SYS_HITL 1 active?'
        if self.rtf_ok_since is None or time.monotonic() - self.rtf_ok_since < RTF_HOLD:
            return False, 'real time factor %.3f, need %.2f for %.0f s' % (
                self.rtf, self.min_rtf, RTF_HOLD)
        return True, 'ready'

    def _publish_status(self):
        ready, reason = self._ready()
        self.status_pub.publish(String(data=json.dumps({
            'ready': ready, 'reason': reason,
            'rtf': None if math.isnan(self.rtf) else round(self.rtf, 4),
            'params': self.px4_params})))
        return ready, reason

    def _heartbeat_and_status(self):
        if self.link is not None:
            # PX4 starts the USB MAVLink instance on a heartbeat (Classic's SendHeartbeat).
            self.link.send(mavlink.MAVLink_heartbeat_message(
                mavlink.MAV_TYPE_GENERIC, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3))
        was = getattr(self, '_was_ready', None)
        ready, reason = self._publish_status()
        # rclpy pins one severity per call site, so ready and not-ready log from separate lines.
        if ready != was and ready:
            self.get_logger().info('HITL ready (RTF %.3f)' % self.rtf)
        elif ready != was:
            self.get_logger().warn('HITL not ready: %s (RTF %.3f)' % (reason, self.rtf))
        elif not ready and int(time.monotonic()) % 5 == 0:
            self.get_logger().warn('HITL not ready: %s' % reason)
        self._was_ready = ready


def main():
    rclpy.init()
    node = HitlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.link is not None:
            node.link.stop = True
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
