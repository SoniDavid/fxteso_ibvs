#!/usr/bin/env python3
"""One row per bag: the numbers that decide whether a tuning change helped.

    python3 summary.py bags/ibvs_... bags/px4_...

  lock%       locked frames over all frames, plus dropouts longer than 0.5 s. The acceptance
              test. Percentage, not time-to-first-loss: one dropped frame is not the same
              failure as losing the target for good.
  roll/pitch  attitude tracking error, deg (commanded vs achieved)
  mu11 sd     noise in the qpsi numerator - the moment the yaw channel is built on
  dist drift  how far ibvs_dist.w moved. A pure integrator, so this is the yaw drift's
              direct cause; the sign is random per run, the magnitude is not.
  err sd      estimation_error_fxt.w, the noise that integrator is walking on
"""
import math
import statistics as st
import sys

from bagread import is_locked, locked_window, read

TOPICS = ['/desired_attitude', '/quad_attitude', '/u_coordinates', '/n_coordinates',
          '/ImFeat_vector', '/estimation_error_fxt', '/ibvs_dist']


def lock_fraction(bag):
    """Locked frames over all recorded frames, and dropouts longer than half a second."""
    events = [(t, is_locked(m)) for t, _, m in read(bag, ['/ImFeat_vector'])]
    if not events:
        return 0.0, 0
    held = sum(1 for _, ok in events if ok)
    drops, run, start = 0, 0, None
    for t, ok in events:
        if not ok:
            run += 1
            if run == 1:
                start = t
        else:
            if run and t - start > 0.5:
                drops += 1
            run = 0
    if run and events[-1][0] - start > 0.5:
        drops += 1
    return 100.0 * held / len(events), drops


def row(bag):
    lo, hi = locked_window(bag)
    des, act, u, n, lk, err, dist = [], [], {}, {}, {}, [], []
    for t, topic, m in read(bag, TOPICS):
        if not lo <= t <= hi:
            continue
        k = round(t, 3)
        if topic == '/desired_attitude':
            des.append((t, (m.x, m.y)))
        elif topic == '/quad_attitude':
            act.append((t, (m.x, m.y)))
        elif topic == '/u_coordinates':
            u[k] = (m.x, m.y, m.z, m.w)
        elif topic == '/n_coordinates':
            n[k] = (m.x, m.y, m.z, m.w)
        elif topic == '/ImFeat_vector':
            lk[k] = is_locked(m)
        elif topic == '/estimation_error_fxt':
            err.append(m.w)
        else:
            dist.append(m.w)

    # attitude tracking error, on a common 20 ms grid
    def held(series, grid):
        out, i, cur = [], 0, None
        for t in grid:
            while i < len(series) and series[i][0] <= t:
                cur = series[i][1]
                i += 1
            out.append(cur)
        return out

    grid = [lo + k * 0.02 for k in range(int((hi - lo) / 0.02))]
    D, A = held(des, grid), held(act, grid)
    keep = [i for i in range(len(grid)) if D[i] and A[i]]
    trk = []
    for axis in (0, 1):
        trk.append(st.pstdev([D[i][axis] - A[i][axis] for i in keep]) * 57.2958
                   if len(keep) > 10 else float('nan'))

    mu11 = []
    for k in sorted(set(u) & set(n)):
        if not lk.get(k, True):
            continue
        us, ns = u[k], n[k]
        ug, ng = sum(us) / 4.0, sum(ns) / 4.0
        mu11.append(sum((x - ug) * (y - ng) for x, y in zip(us, ns)))

    pct, drops = lock_fraction(bag)
    return (pct, drops, trk[0], trk[1],
            st.pstdev(mu11) if len(mu11) > 2 else float('nan'),
            (dist[-1] - dist[0]) if len(dist) > 2 else float('nan'),
            st.pstdev(err) if len(err) > 2 else float('nan'))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    print('%-34s %7s %7s %7s %7s %9s %11s %9s'
          % ('bag', 'lock%', 'drops', 'roll', 'pitch', 'mu11 sd', 'dist drift', 'err sd'))
    for bag in sys.argv[1:]:
        try:
            pct, drops, r, p, m, dr, e = row(bag)
            print('%-34s %6.1f%% %7d %7.2f %7.2f %9.0f %+11.4f %9.5f'
                  % (bag.split('/')[-1], pct, drops, r, p, m, dr, e))
        except SystemExit as exc:
            print('%-34s %s' % (bag.split('/')[-1], exc))
