#!/usr/bin/env python3
"""Derive the per-camera layout variants from the canonical fxteso_ibvs.json: the Vision tab
shows image_distorted or image_raw, whichever the preset's detector consumes.

    python3 src/quad_utils/foxglove/variants.py          # rewrite the derived variants
    python3 src/quad_utils/foxglove/variants.py --list   # which preset wants which layout
"""
import argparse
import collections
import io
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CANONICAL = os.path.join(HERE, 'fxteso_ibvs.json')


def cameras_yaml():
    """quad_description's preset table: from ament, else the source tree (no ROS sourced)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('quad_description'), 'config',
                            'cameras.yaml')
    except Exception:
        return os.path.join(HERE, os.pardir, os.pardir, 'quad_description', 'config',
                            'cameras.yaml')

# The one panel that differs. The canonical file is hand-edited; DERIVED is generated from it and
# must never contain it (generating a file from itself truncates it).
VISION_PANEL = 'Image!camera_vision'
CANONICAL_TOPIC = '/quad/camera/image_distorted'
DERIVED = collections.OrderedDict([
    ('fxteso_ibvs_raw.json', '/quad/camera/image_raw'),
])


def presets_by_topic():
    """preset name -> the image topic image_features is remapped onto."""
    with io.open(cameras_yaml(), encoding='utf-8') as fh:
        cams = yaml.safe_load(fh)['cameras']
    out = collections.OrderedDict()
    for name, cam in cams.items():
        distorted = any(cam.get('distortion') or [])
        out[name] = '/quad/camera/image_distorted' if distorted else '/quad/camera/image_raw'
    return out


def layout_for(preset):
    """The layout filename to import into Studio for this camera preset."""
    topic = presets_by_topic()[preset]
    if topic == CANONICAL_TOPIC:
        return os.path.basename(CANONICAL)
    for filename, want in DERIVED.items():
        if want == topic:
            return filename
    raise KeyError(preset)


def render(topic):
    """The canonical layout, as text, with the Vision tab's image topic swapped."""
    with io.open(CANONICAL, encoding='utf-8') as fh:
        layout = json.load(fh, object_pairs_hook=collections.OrderedDict)
    layout['configById'][VISION_PANEL]['imageMode']['imageTopic'] = topic
    return json.dumps(layout, indent=2) + '\n'


def stale():
    """Derived files that do not match what render() would write."""
    out = []
    for filename, topic in DERIVED.items():
        want = render(topic)
        try:
            with io.open(os.path.join(HERE, filename), encoding='utf-8') as fh:
                have = fh.read()
        except IOError:
            out.append('%s: missing - run variants.py' % filename)
            continue
        if have != want:
            out.append('%s: out of date - run variants.py' % filename)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--list', action='store_true',
                    help='print the preset -> layout mapping and exit')
    args = ap.parse_args()

    if args.list:
        for preset, topic in presets_by_topic().items():
            print('%-24s %-30s %s' % (preset, topic, layout_for(preset)))
        return 0

    # Render every derived file BEFORE writing any of them: CANONICAL is the input.
    rendered = [(name, render(topic)) for name, topic in DERIVED.items()]
    for filename, text in rendered:
        with io.open(os.path.join(HERE, filename), 'w', encoding='utf-8') as fh:
            fh.write(text)
        print('wrote %s  (from %s)' % (filename, os.path.basename(CANONICAL)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
