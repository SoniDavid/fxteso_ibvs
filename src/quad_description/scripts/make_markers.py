#!/usr/bin/env python3
"""Generate the ArUco marker textures for an alternative dictionary.

    python3 make_markers.py 4x4                 # writes models/aruco_target/meshes_4x4/
    python3 make_markers.py single_marker 4x4   # writes models/aruco_single_marker/meshes_4x4/
    python3 make_markers.py single_marker 7x7 --print   # + print/single_marker_7x7.pdf, true size

Filenames must match the original: Target2.dae references the JPEGs by bare name.
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
SINGLE_MARKER = os.path.join(HERE, '..', 'models', 'aruco_single_marker')

# One marker on US letter paper, quiet zone one module wide; camera_presets.target_geometry agrees.
SINGLE_MARKER_ID = 4
US_LETTER_SHEET = (0.2159, 0.2794)
BITS = {'4x4': 4, '5x5': 5, '6x6': 6, '7x7': 7}
SHEET_PX = 960
# Print module, px: ~600 dpi on letter. Any integer works; the PDF's dpi carries the true size.
PRINT_MODULE_PX = 460

SINGLE_MARKER_DAE = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_images>
    <image id="sheet_jpg" name="sheet_jpg"><init_from>single_marker.jpg</init_from></image>
  </library_images>
  <library_effects>
    <effect id="sheet-effect">
      <profile_COMMON>
        <newparam sid="sheet-surface"><surface type="2D"><init_from>sheet_jpg</init_from></surface></newparam>
        <newparam sid="sheet-sampler"><sampler2D><source>sheet-surface</source></sampler2D></newparam>
        <technique sid="common">
          <phong>
            <emission><color sid="emission">0 0 0 1</color></emission>
            <ambient><color sid="ambient">0 0 0 1</color></ambient>
            <diffuse><texture texture="sheet-sampler" texcoord="UVMap"/></diffuse>
            <specular><color sid="specular">0.5 0.5 0.5 1</color></specular>
            <shininess><float sid="shininess">50</float></shininess>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="sheet-material" name="sheet"><instance_effect url="#sheet-effect"/></material>
  </library_materials>
  <library_geometries>
    <geometry id="sheet-mesh" name="sheet">
      <mesh>
        <source id="sheet-pos">
          <float_array id="sheet-pos-array" count="12">{x0} {y0} 0.0005 {x1} {y0} 0.0005 {x1} {y1} 0.0005 {x0} {y1} 0.0005</float_array>
          <technique_common><accessor source="#sheet-pos-array" count="4" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <source id="sheet-nrm">
          <float_array id="sheet-nrm-array" count="3">0 0 1</float_array>
          <technique_common><accessor source="#sheet-nrm-array" count="1" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <source id="sheet-uv">
          <float_array id="sheet-uv-array" count="8">0 0 1 0 1 1 0 1</float_array>
          <technique_common><accessor source="#sheet-uv-array" count="4" stride="2">
            <param name="S" type="float"/><param name="T" type="float"/>
          </accessor></technique_common>
        </source>
        <vertices id="sheet-vtx"><input semantic="POSITION" source="#sheet-pos"/></vertices>
        <triangles material="sheet-material" count="2">
          <input semantic="VERTEX" source="#sheet-vtx" offset="0"/>
          <input semantic="NORMAL" source="#sheet-nrm" offset="1"/>
          <input semantic="TEXCOORD" source="#sheet-uv" offset="2" set="0"/>
          <p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="sheet" name="sheet" type="NODE">
        <instance_geometry url="#sheet-mesh" name="sheet">
          <bind_material><technique_common>
            <instance_material symbol="sheet-material" target="#sheet-material">
              <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
            </instance_material>
          </technique_common></bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
"""


def _sheet(name, module):
    """The whole sheet, white, the marker centred with a one-module quiet zone: (canvas, ink px)."""
    bits = BITS[name]
    width = module * (bits + 4)
    height = int(round(width * US_LETTER_SHEET[1] / US_LETTER_SHEET[0]))
    ink = module * (bits + 2)
    dictionary = cv2.aruco.getPredefinedDictionary(DICTS[name])
    canvas = np.full((height, width), 255, dtype=np.uint8)
    top = (height - ink) // 2
    canvas[top:top + ink, module:module + ink] = cv2.aruco.drawMarker(dictionary, SINGLE_MARKER_ID,
                                                                      ink)
    return canvas, ink


def single_marker(name):
    """The sheet as one Gazebo texture."""
    canvas, ink = _sheet(name, SHEET_PX // (BITS[name] + 4))
    height, width = canvas.shape

    out = os.path.normpath(os.path.join(SINGLE_MARKER, 'meshes_%s' % name))
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, 'single_marker.jpg')
    cv2.imwrite(path, canvas, [cv2.IMWRITE_JPEG_QUALITY, 100])
    w, h = US_LETTER_SHEET
    with open(os.path.join(out, 'SingleMarker.dae'), 'w') as fh:
        fh.write(SINGLE_MARKER_DAE.format(x0=-w / 2, x1=w / 2, y0=-h / 2, y1=h / 2))
    print('wrote %s (%dx%d px, ink %.1f mm, ID %d) and SingleMarker.dae'
          % (path, width, height, 1000 * US_LETTER_SHEET[0] * ink / width, SINGLE_MARKER_ID))


def single_marker_print(name):
    """One US letter page at true size: the dpi is set so the image spans exactly 215.9 mm, which
    makes the ink exactly the size camera_presets.target_geometry assumes. Bilevel, so lossless."""
    from PIL import Image  # print only; the textures need cv2 alone
    canvas, ink = _sheet(name, PRINT_MODULE_PX)
    width = canvas.shape[1]
    dpi = width / (US_LETTER_SHEET[0] / 0.0254)
    out = os.path.normpath(os.path.join(SINGLE_MARKER, 'print'))
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, 'single_marker_%s.pdf' % name)
    Image.fromarray(canvas).convert('1').save(path, 'PDF', resolution=dpi)
    print('wrote %s (%.1f dpi, ink %.1f mm, ID %d): print at actual size, not fit to page'
          % (path, dpi, 1000 * US_LETTER_SHEET[0] * ink / width, SINGLE_MARKER_ID))


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'single_marker':
        name = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] != '--print' else '4x4'
        if name not in DICTS:
            raise SystemExit('dictionary %r is not one of %s' % (name, ', '.join(DICTS)))
        single_marker(name)
        if '--print' in sys.argv:
            single_marker_print(name)
        return
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
