#!/usr/bin/env python3
"""Rebuild the qpsi moment computation from the logged virtual-frame marker points.

image_features.cpp forms, over the four ArUco centroids after the virtual-camera de-rotation:

    mu20 = sum (u_i - ug)^2
    mu02 = sum (n_i - ng)^2
    mu11 = sum (u_i - ug)(n_i - ng)
    qpsi = -0.5 * atan(2*mu11 / (mu20 - mu02))

Two failures need different fixes, so this separates them: a small denominator (markers on a
square -> mu20 == mu02, arctangent of noise over noise) is target geometry; noisy moments are
tuning.

    python3 image_moments.py bags/px4_... bags/ibvs_...

u_coordinates / n_coordinates are logged in pixels (image_features divides by pixel_size).
"""
import statistics as st
import sys

from bagread import is_locked, locked_window, read

TOPICS = ['/u_coordinates', '/n_coordinates', '/ImFeat_vector']


def analyse(bag):
    lo, hi = locked_window(bag)
    u, n, locked = {}, {}, {}
    for t, topic, msg in read(bag, TOPICS):
        if not lo <= t <= hi:
            continue
        key = round(t, 3)
        if topic == '/ImFeat_vector':
            locked[key] = is_locked(msg)
        elif topic == '/u_coordinates':
            u[key] = (msg.x, msg.y, msg.z, msg.w)
        else:
            n[key] = (msg.x, msg.y, msg.z, msg.w)

    mu20, mu02, mu11, den = [], [], [], []
    for key in sorted(set(u) & set(n)):
        if not locked.get(key, True):
            continue
        us, ns = u[key], n[key]
        ug, ng = sum(us) / 4.0, sum(ns) / 4.0
        a = sum((x - ug) ** 2 for x in us)
        b = sum((y - ng) ** 2 for y in ns)
        mu20.append(a)
        mu02.append(b)
        mu11.append(sum((x - ug) * (y - ng) for x, y in zip(us, ns)))
        den.append(a - b)

    if len(den) < 2:
        print('  no locked frames with coordinates')
        return

    scale = st.mean(mu20) + st.mean(mu02)
    print('  locked window %.1f .. %.1f s  (%d frames)' % (lo, hi, len(den)))
    for name, vals in (('mu20', mu20), ('mu02', mu02), ('mu11', mu11), ('mu20-mu02', den)):
        print('  %-12s mean %12.1f   sd %10.1f' % (name, st.mean(vals), st.pstdev(vals)))
    print('  conditioning |mu20-mu02| / (mu20+mu02) = %.4f'
          '   (0 would be the degenerate square case)' % (abs(st.mean(den)) / scale))
    near = sum(1 for d in den if abs(d) < 0.05 * scale)
    print('  frames with |denominator| under 5%% of scale: %d / %d (%.1f%%)'
          % (near, len(den), 100.0 * near / len(den)))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for bag in sys.argv[1:]:
        print('\n===== %s' % bag)
        analyse(bag)
