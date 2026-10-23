"""The camera preset table and everything derived from it: one implementation for the rendered
camera, the Pi driver and image_features, which must never disagree. Loaded by path (importlib).
"""
import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory

PKG = 'quad_description'

# Camera Module 3 Wide at 2304x1296. The widest preset that still clears the 50 Hz loop, which
# is what gives the acquisition transient the field of view it needs.
DEFAULT_CAMERA = 'module3wide_2304'

# image_features.cpp's nominal focal length. Only pixel_size/fx enters the feature model, so
# this cancels everywhere except aD - which is exactly why aD is computable.
FOCAL_LENGTH = 0.00304

# mu20+mu02 of the marker centroids at unit scale and depth: 4*(0.225^2 + 0.300^2).
CENTROID_MOMENT = 0.5625

# Marker ink, black edge to black edge, at target_scale 1.0. The printed 0.5 target has 122.2 mm.
MARKER_INK = 0.2445

# target_layout single_marker: one marker on US letter paper, quiet zone one module wide inside it.
US_LETTER_SHEET = (0.2159, 0.2794)
DICT_BITS = {'4x4': 4, '5x5': 5, '6x6': 6, '7x7': 7}
LAYOUTS = ('four_markers', 'single_marker')

# The rate image_features and fixed_eso both run at.
LOOP_HZ = 50.0


def table():
    """The preset table from config/cameras.yaml."""
    path = os.path.join(get_package_share_directory(PKG), 'config', 'cameras.yaml')
    with open(path) as fh:
        return yaml.safe_load(fh)['cameras']


def target_geometry(target_layout='four_markers', target_scale=1.0, marker_dict='7x7'):
    """mu20+mu02 of the feature points at unit depth, and the marker ink side. single_marker is
    a fixed sheet, so target_scale does not apply to it; its points are the corners, 2*ink^2."""
    if target_layout == 'four_markers':
        return {'moment': CENTROID_MOMENT * target_scale ** 2, 'ink': MARKER_INK * target_scale}
    if target_layout == 'single_marker':
        if marker_dict not in DICT_BITS:
            raise RuntimeError('marker_dict:=%s is not one of %s.'
                               % (marker_dict, ', '.join(DICT_BITS)))
        bits = DICT_BITS[marker_dict]
        ink = US_LETTER_SHEET[0] * (bits + 2) / (bits + 4)
        return {'moment': 2.0 * ink ** 2, 'ink': ink}
    raise RuntimeError('target_layout:=%s is not one of %s.' % (target_layout, ', '.join(LAYOUTS)))


def resolve(camera, target_scale=1.0, zD=2.5, target_layout='four_markers', marker_dict='7x7'):
    """Everything the launch files need for one (preset, target, servoing depth).
    aD = moment * (f/zD)^2; aD, not zD, decides where the aircraft settles."""
    presets = table()
    if camera not in presets:
        raise RuntimeError('camera:=%s is not one of %s.' % (camera, ', '.join(presets)))
    if target_scale <= 0:
        raise RuntimeError('target_scale:=%g must be positive.' % target_scale)
    if zD <= 0:
        raise RuntimeError('zD:=%g must be positive.' % zD)

    cam = presets[camera]
    width, height = cam['render']
    hfov = math.radians(cam['hfov_deg'])
    return {
        'name': camera,
        'hfov': hfov,
        'width': width,
        'height': height,
        'distortion': list(cam['distortion']),
        'fx': width / (2.0 * math.tan(hfov / 2.0)),
        'fps': cam['fps'],
        'native': list(cam['native']),
        'aD': target_geometry(target_layout, target_scale, marker_dict)['moment']
              * (FOCAL_LENGTH / zD) ** 2,
    }


def marker_px(cam, target_scale, zD, target_layout='four_markers', marker_dict='7x7'):
    """Nominal marker side in pixels at the servoing depth, at the width cam['fx'] was sized for."""
    return target_geometry(target_layout, target_scale, marker_dict)['ink'] * cam['fx'] / zD


def min_marker_ratio(cam, target_scale, zD, margin=0.7, target_layout='four_markers',
                     marker_dict='7x7'):
    """aruco3's minMarkerLengthRatioOriginalImg: margin x the nominal marker, over the long side."""
    return (margin * marker_px(cam, target_scale, zD, target_layout, marker_dict)
            / max(cam['width'], cam['height']))


def summary(cam, zD, camera_rate=0.0):
    """One line for the launch log. The frame rate is here because it is a hardware limit the
    sim otherwise hides: Module 2's only full-FOV mode runs at 41.85 fps against a 50 Hz loop."""
    slow = '' if cam['fps'] >= LOOP_HZ else '  <- BELOW the %.0f Hz loop' % LOOP_HZ
    rendered = '' if camera_rate <= 0 else ', rendered at %g fps' % camera_rate
    return ('camera %s: %.1f deg hFOV, fx %.1f px, render %dx%d, hardware mode %dx%d @ %.1f fps%s'
            '%s; zD %.2f m, aD %.4e'
            % (cam['name'], math.degrees(cam['hfov']), cam['fx'], cam['width'], cam['height'],
               cam['native'][0], cam['native'][1], cam['fps'], slow, rendered, zD, cam['aD']))
