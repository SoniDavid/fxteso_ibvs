"""Synthetic inputs for the hardware benchmark - stand-in for PX4 and the plant.

Publishes small bounded non-zero sinusoids on the topics no in-graph node produces, so
the estimation/control loops run their full arithmetic without a simulator or a Pixhawk:

  100 Hz  /quad_position /quad_attitude /quad_attitude_velocity   (geometry_msgs/Vector3)
   50 Hz  /tgt_velocity /tgt_acceleration                         (geometry_msgs/Vector3)
   50 Hz  /tgt_yaw_rate                                           (std_msgs/Float64)

With feed_vision:=true it also fakes a marker lock so fixed_eso / pos_ctrl (gated on
ImFeat_valid) run:

   50 Hz  /ImFeat_vector   (geometry_msgs/Quaternion, small wobble around 0,0,1,0)
   50 Hz  /ImFeat_valid    (std_msgs/Bool = true)
   50 Hz  /a_value         (std_msgs/Float64)

Not a plant model - the values are meaningless, only their shape and rate matter. Run
with use_sim_time:=false.
"""

import math
import os

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Bool, Float64

_NUM = ParameterDescriptor(dynamic_typing=True)


class BenchFeeder(Node):

    def __init__(self):
        super().__init__('bench_feeder')
        self._feed_vision = bool(self.declare_parameter('feed_vision', False).value)
        fast = float(self.declare_parameter('rate_hz_fast', 100.0, _NUM).value)
        slow = float(self.declare_parameter('rate_hz_slow', 50.0, _NUM).value)
        self._amp = float(self.declare_parameter('amplitude', 0.1, _NUM).value)

        self._pos = self.create_publisher(Vector3, '/quad_position', 10)
        self._att = self.create_publisher(Vector3, '/quad_attitude', 10)
        self._attvel = self.create_publisher(Vector3, '/quad_attitude_velocity', 10)
        self._tgt_vel = self.create_publisher(Vector3, '/tgt_velocity', 10)
        self._tgt_acc = self.create_publisher(Vector3, '/tgt_acceleration', 10)
        self._tgt_yr = self.create_publisher(Float64, '/tgt_yaw_rate', 10)

        self._imfeat = self._imvalid = self._aval = None
        if self._feed_vision:
            self._imfeat = self.create_publisher(Quaternion, '/ImFeat_vector', 10)
            self._imvalid = self.create_publisher(Bool, '/ImFeat_valid', 10)
            self._aval = self.create_publisher(Float64, '/a_value', 10)

        self._t = 0.0
        self._dt_fast = 1.0 / fast
        self.create_timer(self._dt_fast, self._tick_fast)
        self.create_timer(1.0 / slow, self._tick_slow)
        self.get_logger().info(
            'bench_feeder: %.0f/%.0f Hz, amplitude %.2f, feed_vision=%s'
            % (fast, slow, self._amp, self._feed_vision))

    def _v3(self, a, b, c):
        m = Vector3()
        m.x, m.y, m.z = float(a), float(b), float(c)
        return m

    def _tick_fast(self):
        self._t += self._dt_fast
        w = 2.0 * math.pi * 0.2 * self._t          # 0.2 Hz wobble
        s, c = self._amp * math.sin(w), self._amp * math.cos(w)
        self._pos.publish(self._v3(s, c, 1.2 + 0.1 * s))
        self._att.publish(self._v3(0.5 * s, 0.5 * c, 0.2 * s))
        self._attvel.publish(self._v3(0.3 * c, -0.3 * s, 0.1 * c))

    def _tick_slow(self):
        w = 2.0 * math.pi * 0.2 * self._t
        s, c = self._amp * math.sin(w), self._amp * math.cos(w)
        self._tgt_vel.publish(self._v3(0.2 * s, 0.2 * c, 0.0))
        self._tgt_acc.publish(self._v3(0.1 * c, -0.1 * s, 0.0))
        yr = Float64()
        yr.data = 0.05 * s
        self._tgt_yr.publish(yr)

        if self._feed_vision:
            q = Quaternion()
            q.x, q.y, q.z, q.w = 0.3 * s, 0.3 * c, 1.0, 0.2 * s
            self._imfeat.publish(q)
            b = Bool()
            b.data = True
            self._imvalid.publish(b)
            a = Float64()
            a.data = 7.66e-7 * (1.0 + 0.05 * s)
            self._aval.publish(a)


def main(args=None):
    # See camera_node.py's main() - keeps this participant consistent with the rest of
    # the graph even though its own topics are small. Must precede rclpy.init().
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    rclpy.init(args=args)
    node = BenchFeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
