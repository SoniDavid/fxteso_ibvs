#!/usr/bin/env python3
"""Publish one depth target to pos_ctrl and confirm it took.

  ros2 run quad_px4 zd_set.py 0.8          # hardware (LARGE_DATA, as hardware.launch)
  ros2 run quad_px4 zd_set.py 0.8 --sitl   # SITL (default transport)

pos_ctrl refuses it outside [zD_min, zD_max] and before consent. Not sent while
/px4_offboard_bridge/in_control is false; resent until /zD_active echoes it.
"""
import os
import sys

if '--sitl' not in sys.argv:
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, Float64  # noqa: E402

DISCOVERY_S = 10.0   # LARGE_DATA discovery under flight load takes seconds
CONFIRM_S = 5.0      # total time to see the echo
RESEND_S = 1.0


def main():
    sitl = '--sitl' in sys.argv
    args = [a for a in sys.argv[1:] if a != '--sitl']
    if len(args) != 1:
        sys.exit(__doc__)
    target = float(args[0])
    rclpy.init()
    node = Node('zd_set')
    clock = node.get_clock()

    def now():
        return clock.now().nanoseconds * 1e-9

    pub = node.create_publisher(Float64, '/zD_setpoint', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE))
    active = []
    node.create_subscription(Float64, '/zD_active', lambda m: active.append(m.data), 10)
    in_control = []
    node.create_subscription(Bool, '/px4_offboard_bridge/in_control',
                             lambda m: in_control.append(m.data), QoSProfile(
                                 depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def done(msg):
        # Skip DDS teardown: it aborts/segfaults ~1 in 5 while the 50 Hz echo is still arriving.
        print(msg, flush=True)
        os._exit(0)

    # One send reaches only the readers matched at that instant (the bag recorder got
    # them instead of pos_ctrl in flight), so wait for pos_ctrl and every discovered reader.
    deadline = now() + DISCOVERY_S
    while True:
        rclpy.spin_once(node, timeout_sec=0.1)
        readers = node.get_subscriptions_info_by_topic('/zD_setpoint')
        has_ctrl = any(r.node_name == 'pos_ctrl' for r in readers)
        if has_ctrl and pub.get_subscription_count() >= len(readers) \
                and (in_control or sitl):
            break
        if now() > deadline:
            if not has_ctrl:
                done('pos_ctrl not discovered on /zD_setpoint after %.0f s - is it running, '
                     'same domain/transport?' % DISCOVERY_S)
            if not in_control:
                done('no /px4_offboard_bridge/in_control after %.0f s - not sent' % DISCOVERY_S)
            break   # some other reader never matched; pos_ctrl's echo still confirms
    for _ in range(5):  # let the reader side finish matching too
        rclpy.spin_once(node, timeout_sec=0.1)

    # Never queue a target across the hold: a retry landing at consent would be a step.
    if in_control and not in_control[-1]:
        done('not in control - not sent (pos_ctrl holds zD until the handover)')

    end = now() + CONFIRM_S
    next_send = 0.0
    sends = 0
    while now() < end:
        if in_control and not in_control[-1]:
            done('lost control after %d send(s) - pos_ctrl resets zD in the hold' % sends)
        if active and abs(active[-1] - target) < 1e-3:
            done('zD_active %.3f m - accepted (%d send(s))' % (active[-1], sends))
        if now() >= next_send:
            active.clear()   # only echoes after this send count
            pub.publish(Float64(data=target))
            sends += 1
            if sends == 1:
                print('sent zD %.3f m' % target)
            next_send = now() + RESEND_S
        rclpy.spin_once(node, timeout_sec=0.05)

    if not active:
        done('no /zD_active seen in %.0f s - check the pos_ctrl log' % CONFIRM_S)
    done('zD_active %.3f m - NOT CONFIRMED after %d sends (outside [zD_min, zD_max]? '
         'see the pos_ctrl log)' % (active[-1], sends))


if __name__ == '__main__':
    main()
