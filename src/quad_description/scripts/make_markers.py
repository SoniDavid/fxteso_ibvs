#!/usr/bin/env python3
"""Generate the ArUco marker textures for an alternative dictionary.

    python3 make_markers.py 4x4          # writes models/aruco_target/meshes_4x4/

gz_sim.launch.py's marker_dict:= argument symlinks the derived aruco_target's meshes/ at the
directory this writes, so the filenames and the Target2.dae beside them must match the original
exactly - the DAE references the JPEGs by bare filename.

"""
import os
import shutil
import sys

import cv2
import numpy as np

IDS = (4, 6, 8, 10)
SIZE = 800           # texture side, px; the originals are ~810, and only the ratio matters
INK_FRACTION = 0.815
DICTS = {'4x4': cv2.aruco.DICT_4X4_50, '5x5': cv2.aruco.DICT_5X5_50,
         '6x6': cv2.aruco.DICT_6X6_50, '7x7': cv2.aruco.DICT_7X7_50}

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES = os.path.join(HERE, '..', 'models', 'aruco_target', 'meshes')


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else '4x4'
    if name not in DICTS:
        raise SystemExit('dictionary %r is not one of %s' % (name, ', '.join(DICTS)))

    out = os.path.normpath(os.path.join(MESHES, '..', 'meshes_%s' % name))
    os.makedirs(out, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(DICTS[name])

    ink = int(round(SIZE * INK_FRACTION))
    pad = (SIZE - ink) // 2
    for marker_id in IDS:
        img = cv2.aruco.drawMarker(dictionary, marker_id, ink)
        canvas = np.full((SIZE, SIZE), 255, dtype=np.uint8)
        canvas[pad:pad + ink, pad:pad + ink] = img
        path = os.path.join(out, 'aruco_marker_%02d.jpg' % marker_id)
        # Quality 100: JPEG ringing on a marker edge is a decode failure, not a cosmetic one.
        cv2.imwrite(path, canvas, [cv2.IMWRITE_JPEG_QUALITY, 100])
        print('wrote %s (%d px texture, %d px ink)' % (path, SIZE, ink))

    # The DAE is geometry and UVs only - identical for every dictionary - but it has to sit
    # beside the JPEGs it names, because the derived model symlinks this whole directory.
    dae = os.path.join(os.path.normpath(MESHES), 'Target2.dae')
    shutil.copy(dae, out)
    print('copied %s' % os.path.join(out, 'Target2.dae'))


if __name__ == '__main__':
    main()
