#!/usr/bin/env python3
"""How well the inner loop tracks the attitude pos_ctrl asks for, and with what lag.

The objective for tuning PX4's attitude and rate gains. plant:=analytic is the reference:
the ANFTSMC holds roll and pitch to ~0.08 deg at ~80 ms.

    python3 attitude_tracking.py bags/px4_... bags/ibvs_...
"""
import statistics as st
import sys

from bagread import locked_window, read

TOPICS = ['/desired_attitude', '/quad_attitude']
DT = 0.02          # pos_ctrl runs at 50 Hz, so this is one command per sample
MAX_LAG = 30       # samples, i.e. 600 ms - beyond that the correlation is meaningless


def resample(samples, grid):
    """Zero-order hold onto a uniform grid; both topics are published at their own rate."""
    out, i, held = [], 0, None
    for t in grid:
        while i < len(samples) and samples[i][0] <= t:
            held = samples[i][1]
            i += 1
        out.append(held)
    return out


def correlation(a, b):
    ma, mb = st.mean(a), st.mean(b)
    na = sum((x - ma) ** 2 for x in a) ** 0.5
    nb = sum((y - mb) ** 2 for y in b) ** 0.5
    if na == 0.0 or nb == 0.0:
        return 0.0
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (na * nb)


def analyse(bag):
    lo, hi = locked_window(bag)
    commanded, actual = [], []
    for t, topic, msg in read(bag, TOPICS):
        if not lo <= t <= hi:
            continue
        (commanded if topic == '/desired_attitude' else actual).append((t, (msg.x, msg.y)))
    if len(commanded) < 50 or len(actual) < 50:
        print('  too few samples in the locked window')
        return

    grid = [lo + k * DT for k in range(int((hi - lo) / DT))]
    cmd = resample(commanded, grid)
    act = resample(actual, grid)
    keep = [i for i in range(len(grid)) if cmd[i] is not None and act[i] is not None]

    print('  locked window %.1f .. %.1f s' % (lo, hi))
    for axis, name in ((0, 'roll'), (1, 'pitch')):
        c = [cmd[i][axis] for i in keep]
        a = [act[i][axis] for i in keep]
        err = st.pstdev([x - y for x, y in zip(c, a)])
        # Shifting the achieved series back against the commanded one: the shift that
        # correlates best is the loop's transport delay.
        lag = max(range(MAX_LAG),
                  key=lambda k: correlation(c[:len(c) - k], a[k:]) if len(c) - k > 10 else -2)
        best = correlation(c[:len(c) - lag], a[lag:])
        print('  %-6s cmd sd %.4f  act sd %.4f  tracking err sd %.4f rad (%.2f deg)  '
              'lag %3.0f ms (r=%.3f)'
              % (name, st.pstdev(c), st.pstdev(a), err, err * 57.2958, lag * DT * 1000, best))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for bag in sys.argv[1:]:
        print('\n===== %s' % bag)
        analyse(bag)
