#!/usr/bin/env python3
"""Noise and bias along the yaw channel, from the image feature to the command.

The yaw channel has no absolute reference - pos_ctrl integrates its command twice - so noise
here random-walks into the commanded yaw. Mean column is drift, sd is noise; the chain runs
top to bottom.

    python3 yaw_channel.py bags/px4_... bags/ibvs_...
"""
import statistics as st
import sys

from bagread import locked_window, read

# topic -> (field, what it is in the loop)
CHAIN = [
    ('/ImFeat_vector', 'w', 'qpsi, measured'),
    ('/ImFeat_estimates_fxt', 'w', 'qpsi, FxTESO estimate -> error(3)'),
    ('/ImFeat_dot_estimates_fxt', 'w', 'qpsi_dot estimate -> error_dot(3)'),
    ('/ibvs_dist', 'w', 'FxTESO total-disturbance estimate'),
    ('/ibvs_control_input', 'w', 'yaw_ddot command  <- drift comes from this mean'),
    ('/adaptive_gain', 'w', 'k(3), the adaptive gain'),
    ('/desired_attitude', 'z', 'commanded yaw'),
    ('/quad_attitude', 'z', 'achieved yaw'),
]


def analyse(bag):
    lo, hi = locked_window(bag)
    topics = [t for t, _, _ in CHAIN]
    series = {t: [] for t in topics}
    for t, topic, msg in read(bag, topics):
        if lo <= t <= hi:
            series[topic].append(msg)

    print('  locked window %.1f .. %.1f s' % (lo, hi))
    print('  %-30s %12s %10s %10s' % ('', 'mean', 'sd', '|max|'))
    for topic, field, what in CHAIN:
        vals = [getattr(m, field) for m in series[topic]]
        if len(vals) < 2:
            continue
        print('  %-30s %+12.5f %10.5f %10.5f   %s'
              % (topic.lstrip('/') + '.' + field, st.mean(vals), st.pstdev(vals),
                 max(abs(v) for v in vals), what))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for bag in sys.argv[1:]:
        print('\n===== %s' % bag)
        analyse(bag)
