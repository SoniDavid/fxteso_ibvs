#!/usr/bin/env python3
"""HITL start gate: exits 0 once hitl_bridge reports ready with the PX4 parameters this terminal
derived, 1 on a mismatch or timeout. hitl_flight.launch.py arms nothing until it does."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('hitl_wait')
    expected = json.loads(node.declare_parameter('px4_params', '{}').value)
    timeout = node.declare_parameter('timeout', 300.0).value
    status = {}

    def on_status(msg):
        status.clear()
        status.update(json.loads(msg.data))

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/hitl_bridge/status', on_status, qos)

    start = time.monotonic()
    last_log = 0.0
    code = 1
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic() - start
        if status:
            pushed = status.get('params', {})
            differ = {k: (v, pushed.get(k)) for k, v in expected.items() if pushed.get(k) != v}
            if differ:
                for name, (want, got) in sorted(differ.items()):
                    node.get_logger().fatal('%s: this terminal derives %s, hitl_bridge pushed %s'
                                            % (name, want, got))
                node.get_logger().fatal('Pass the same arguments to hitl_world.launch.py and '
                                        'hitl_flight.launch.py.')
                break
            if status.get('ready'):
                node.get_logger().info('hitl_bridge ready (RTF %.3f) - starting the flight stack.'
                                       % status.get('rtf', float('nan')))
                code = 0
                break
        if now - last_log >= 5.0:
            last_log = now
            node.get_logger().info('waiting for hitl_bridge: %s'
                                   % (status.get('reason') if status else 'no status yet - is '
                                      'hitl_world.launch.py running?'))
        if timeout > 0 and now > timeout:
            node.get_logger().fatal('hitl_bridge not ready after %.0f s.' % timeout)
            break

    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
