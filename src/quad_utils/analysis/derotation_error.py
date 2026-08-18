#!/usr/bin/env python3
"""The attitude error that actually corrupts the image moments.

image_features.cpp de-rotates with the *estimated* attitude, so what matters is estimate vs
truth - not commanded vs achieved.
Needs /quad_state (Gazebo truth) in the bag. Reports both stages, plus the true attitude's
own motion, since a growing error can mean either a worse estimate or a busier aircraft.

    python3 derotation_error.py bags/px4_... bags/ibvs_...
"""
import math
import statistics as st
import sys

from bagread import locked_window, read

TOPICS = ['/quad_state', '/quad_attitude', '/attitude_estimates']
DT = 0.02


def rpy_from_quat(q):
    """Roll, pitch, yaw about the frame's own axes."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sinr, cosr), math.asin(sinp), math.atan2(siny, cosy)


def resample(samples, grid):
    out, i, held = [], 0, None
    for t in grid:
        while i < len(samples) and samples[i][0] <= t:
            held = samples[i][1]
            i += 1
        out.append(held)
    return out


def analyse(bag):
    lo, hi = locked_window(bag)
    truth, est, filt = [], [], []
    for t, topic, msg in read(bag, TOPICS):
        if not lo <= t <= hi:
            continue
        if topic == '/quad_state':
            r, p, _ = rpy_from_quat(msg.pose.pose.orientation)
            # gz world (ENU, FLU body) -> the workspace's NED/FRD, as gz_state_adapter
            # converts it: (roll, -pitch, -yaw).
            truth.append((t, (r, -p)))
        elif topic == '/quad_attitude':
            est.append((t, (msg.x, msg.y)))
        else:
            filt.append((t, (msg.linear.x, msg.linear.y)))

    if len(truth) < 50:
        print('  no /quad_state in this bag - re-record with the current bag_topics.yaml')
        return

    grid = [lo + k * DT for k in range(int((hi - lo) / DT))]
    T, E, F = resample(truth, grid), resample(est, grid), resample(filt, grid)
    keep = [i for i in range(len(grid)) if T[i] and E[i] and F[i]]

    print('  locked window %.1f .. %.1f s' % (lo, hi))
    for axis, name in ((0, 'roll'), (1, 'pitch')):
        tv = [T[i][axis] for i in keep]
        ev = [E[i][axis] for i in keep]
        fv = [F[i][axis] for i in keep]
        est_err = st.pstdev([a - b for a, b in zip(tv, ev)])
        filt_err = st.pstdev([a - b for a, b in zip(tv, fv)])
        print('  %-6s true motion sd %.4f rad (%.2f deg) | '
              'estimator err %.4f (%.2f deg) | de-rotation err %.4f (%.2f deg)'
              % (name, st.pstdev(tv), st.pstdev(tv) * 57.2958,
                 est_err, est_err * 57.2958, filt_err, filt_err * 57.2958))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for bag in sys.argv[1:]:
        print('\n===== %s' % bag)
        analyse(bag)
