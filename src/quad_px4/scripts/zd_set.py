#!/usr/bin/env python3
"""Publish one depth target to pos_ctrl and confirm it took.

  ros2 run quad_px4 zd_set.py 0.8          # hardware (LARGE_DATA, as hardware.launch)
  ros2 run quad_px4 zd_set.py 0.8 --sitl   # SITL (default transport)

pos_ctrl refuses it outside [zD_min, zD_max] and before consent; the echo of
/zD_active says which.
"""
import os
import sys

if '--sitl' not in sys.argv:
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402


def main():
    args = [a for a in sys.argv[1:] if a != '--sitl']
    if len(args) != 1:
        sys.exit(__doc__)
    target = float(args[0])
    rclpy.init()
    node = Node('zd_set')
    pub = node.create_publisher(Float64, '/zD_setpoint', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE))
    active = []
    node.create_subscription(Float64, '/zD_active', lambda m: active.append(m.data), 10)

    # Discovery over LARGE_DATA is slow; a message sent before the match is lost.
    deadline = node.get_clock().now().nanoseconds + 10e9
    while pub.get_subscription_count() == 0:
        if node.get_clock().now().nanoseconds > deadline:
            sys.exit('no /zD_setpoint subscriber after 10 s - is pos_ctrl running, same domain/transport?')
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(5):  # let the reader side finish matching too
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(Float64(data=target))
    print('sent zD %.3f m' % target)

    active.clear()
    end = node.get_clock().now().nanoseconds + 1.5e9
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not active:
        print('no /zD_active seen - check the pos_ctrl log')
    else:
        ok = abs(active[-1] - target) < 1e-3
        print('zD_active %.3f m - %s' % (active[-1], 'accepted' if ok else 'REFUSED, see the pos_ctrl log'))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
