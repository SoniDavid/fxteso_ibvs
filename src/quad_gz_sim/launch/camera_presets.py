"""The camera preset table and everything derived from it.

One implementation, because the numbers have to reach two places that must never disagree:
the <camera> block gz-sim renders, and image_features' feature model. A mismatch there does
not fail - it silently computes image moments against a camera that was not rendered.

Imported by gz_sim.launch.py directly, and by the top-level launch files (quad_px4's
sitl.launch.py, quad_utils' sim.launch.py) through importlib, since share/<pkg>/launch is not
on sys.path. The offline footprint calculator reads the same YAML, so the two agree by
construction rather than by convention.
"""
import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory

PKG = 'quad_gz_sim'

# Camera Module 3 Wide at 2304x1296. The widest preset that still clears the 50 Hz loop, which
# is what gives the acquisition transient the field of view it needs.
DEFAULT_CAMERA = 'module3wide_2304'

# image_features.cpp's nominal focal length. Only pixel_size/fx enters the feature model, so
# this cancels everywhere except aD - which is exactly why aD is computable.
FOCAL_LENGTH = 0.00304

# Sum of squared marker-centroid offsets on the target plane, 4*(0.225^2 + 0.300^2). This is
# mu20+mu02 at unit scale and unit depth; the quiet zone around each marker does not enter it,
# which is why the derivation below reproduces the flown constant exactly.
CENTROID_MOMENT = 0.5625

# The rate image_features and fixed_eso both run at.
LOOP_HZ = 50.0


def table():
    """The preset table from config/cameras.yaml."""
    path = os.path.join(get_package_share_directory(PKG), 'config', 'cameras.yaml')
    with open(path) as fh:
        return yaml.safe_load(fh)['cameras']


def resolve(camera, target_scale=1.0, zD=2.5):
    """Everything the launch files need for one (preset, target scale, servoing depth).

    aD is derived rather than carried: aD = CENTROID_MOMENT * scale^2 * (f/zD)^2. At scale 1.0
    and 2.605 m this gives 7.6604e-07 against the 7.6589e-07 image_features had hardcoded, so
    the flown constant was aD for 2.605 m - not the 2.5 m the interaction matrix assumes. It is
    aD, not zD, that decides where the aircraft settles.
    """
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
        'aD': CENTROID_MOMENT * target_scale ** 2 * (FOCAL_LENGTH / zD) ** 2,
    }


def summary(cam, zD, camera_rate=0.0):
    """One line for the launch log. The frame rate is here because it is a hardware limit the
    sim otherwise hides: Module 2's only full-FOV mode runs at 41.85 fps against a 50 Hz loop."""
    slow = '' if cam['fps'] >= LOOP_HZ else '  <- BELOW the %.0f Hz loop' % LOOP_HZ
    rendered = '' if camera_rate <= 0 else ', rendered at %g fps' % camera_rate
    return ('camera %s: %.1f deg hFOV, fx %.1f px, render %dx%d, hardware mode %dx%d @ %.1f fps%s'
            '%s; zD %.2f m, aD %.4e'
            % (cam['name'], math.degrees(cam['hfov']), cam['fx'], cam['width'], cam['height'],
               cam['native'][0], cam['native'][1], cam['fps'], slow, rendered, zD, cam['aD']))
