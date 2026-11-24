#!/usr/bin/env python3
"""SITL depth-step driver: once px4_offboard_bridge is in control, waits `settle` s, then publishes
each of `targets` (m) to /zD_setpoint every `dwell` s. Stops for good if in_control drops for > 1 s."""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64


def main():
    rclpy.init()
    node = Node('zd_schedule')
    targets = [float(v) for v in json.loads(node.declare_parameter('targets', '[]').value)]
    settle = node.declare_parameter('settle', 20.0).value
    dwell = node.declare_parameter('dwell', 25.0).value
    log = node.get_logger()

    pub = node.create_publisher(Float64, '/zD_setpoint', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE))
    state = {'ic': False, 'seen': False, 'off': None, 't0': None, 'next': 0, 'done': False}

    def on_in_control(m):
        state['ic'] = m.data
        state['seen'] |= m.data

    node.create_subscription(Bool, '/px4_offboard_bridge/in_control', on_in_control, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def tick():
        now = node.get_clock().now()
        if state['done'] or not state['seen'] or now.nanoseconds == 0:
            return
        if not state['ic']:  # the scorers' rule: an off-gap under 1 s (handover blip) is not a loss
            state['off'] = state['off'] or now
            if (now - state['off']).nanoseconds > 1e9:
                state['done'] = True
                log.warn('in_control dropped - schedule aborted after %d step(s)' % state['next'])
            return
        state['off'] = None
        if state['t0'] is None:  # latched in_control can arrive before /clock; start the timer here
            state['t0'] = now
            log.info('in control - first depth step in %.1f s' % settle)
        t = (now - state['t0']).nanoseconds * 1e-9
        i = state['next']
        if i < len(targets) and t >= settle + i * dwell:
            pub.publish(Float64(data=targets[i]))
            log.info('t+%.1f s: zD setpoint %.3f m' % (t, targets[i]))
            state['next'] += 1
            state['done'] = state['next'] == len(targets)

    log.info('zD schedule %s, settle %.1f s, dwell %.1f s' % (targets, settle, dwell))
    node.create_timer(0.05, tick)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
