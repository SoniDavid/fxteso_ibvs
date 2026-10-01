"""Measure /quad/camera/image_raw delivery: rate, frame age, jitter.

    ros2 run quad_cam probe_latency
    ros2 run quad_cam probe_latency --ros-args -p image_topic:=/quad/camera/image_raw \\
        -p image_qos:=sensor_data

Frame age = receive time (this node's clock) minus the publisher's header stamp, so it
only means anything when both run on the same machine with use_sim_time:=false.
"""

import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image


class ProbeLatency(Node):

    def __init__(self):
        super().__init__("probe_latency")
        topic = self.declare_parameter("image_topic", "/quad/camera/image_raw").value
        qos_kind = self.declare_parameter("image_qos", "reliable").value
        self._report_every = float(self.declare_parameter("report_period_s", 5.0).value)

        reliability = (QoSReliabilityPolicy.BEST_EFFORT if qos_kind == "sensor_data"
                       else QoSReliabilityPolicy.RELIABLE)
        qos = QoSProfile(reliability=reliability, history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Image, topic, self._cb, qos)

        self._ages = []
        self._recv = []
        self._last_recv = None
        self._t0 = time.monotonic()
        self._n = 0
        self.create_timer(self._report_every, self._report)
        self.get_logger().info("probing %s (%s)" % (topic, qos_kind))

    def _cb(self, msg: Image):
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._ages.append((now - stamp) / 1e6)          # ms
        t = time.monotonic()
        if self._last_recv is not None:
            self._recv.append((t - self._last_recv) * 1e3)
        self._last_recv = t
        self._n += 1

    def _report(self):
        if not self._recv:
            self.get_logger().warn("no frames received")
            return
        dt = time.monotonic() - self._t0
        gaps = sorted(self._recv)
        ages = sorted(self._ages)

        def pct(xs, p):
            return xs[min(len(xs) - 1, int(p * len(xs)))]

        self.get_logger().info(
            "rate %.1f Hz | inter-frame gap ms  mean %.1f  p95 %.1f  max %.1f | "
            "frame age ms  mean %.1f  p95 %.1f  max %.1f  (n=%d)"
            % (self._n / dt,
               statistics.mean(gaps), pct(gaps, 0.95), gaps[-1],
               statistics.mean(ages), pct(ages, 0.95), ages[-1], self._n))
        self._recv.clear()
        self._ages.clear()
        self._t0 = time.monotonic()
        self._n = 0


def main(args=None):
    # No transport override: a LARGE_DATA subscriber hears nothing from a default-transport camera.
    # Run with FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA against camera_node.
    rclpy.init(args=args)
    node = ProbeLatency()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
