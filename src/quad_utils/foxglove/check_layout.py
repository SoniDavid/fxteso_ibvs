#!/usr/bin/env python3
"""Check fxteso_ibvs.json against the topics the stack actually publishes.

A message path pointing at something that does not exist draws nothing rather than
erroring, which looks identical to a genuinely flat signal. Run after editing the layout:

    python3 src/quad_utils/foxglove/check_layout.py

Source-tree only: it reads the .cpp files across every sibling package, not the install.
"""
import glob
import json
import os
import re
import sys

import variants

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = os.path.join(HERE, 'fxteso_ibvs.json')
# Publishers are spread across quad_control, quad_gz_sim and quad_utils, so glob every
# sibling package's src/ rather than just this one's.
SRC = os.path.join(HERE, os.pardir, os.pardir, '*', 'src', '*.cpp')

# Only these are addressed by the layout; enough to catch a mistyped field name.
FIELDS = {
    'geometry_msgs::msg::Vector3': {'x', 'y', 'z'},
    'geometry_msgs::msg::Quaternion': {'x', 'y', 'z', 'w'},
    'std_msgs::msg::Float64': {'data'},
    'geometry_msgs::msg::Twist': {'linear', 'angular'},
    'nav_msgs::msg::Path': {'header', 'poses'},
    'visualization_msgs::msg::MarkerArray': {'markers'},
    'std_msgs::msg::Bool': {'data'},
}


def publishers():
    """topic -> message type, from every live create_publisher in the package."""
    found = {}
    for path in glob.glob(SRC):
        # Strip comments first, then match whole-file: some declarations wrap across lines.
        src = re.sub(r'//[^\n]*', '', open(path).read())
        for m in re.finditer(r'create_publisher<([^>]+)>\(\s*"([^"]+)"', src):
            found['/' + m.group(2).lstrip('/')] = m.group(1)
    # Not matched by the regex above: image_raw and camera_info are bridged in from Gazebo
    # (config/bridge.yaml), and camera_distort publishes to a topic held in a parameter rather
    # than a string literal. image_distorted exists only for presets with non-zero distortion
    # (gz_sim.launch.py _distort), which is what image_features is remapped onto - the Vision
    # tab follows it, so on an undistorted preset that one panel is deliberately blank.
    found['/quad/camera/image_raw'] = 'sensor_msgs::msg::Image'
    found['/quad/camera/image_distorted'] = 'sensor_msgs::msg::Image'
    found['/quad/camera/camera_info'] = 'sensor_msgs::msg::CameraInfo'
    return found


def collect_paths(node, out):
    if isinstance(node, dict):
        for key, val in node.items():
            if key == 'paths' and isinstance(val, list):
                out.extend(p['value'] for p in val if 'value' in p)
            elif key == 'xAxisPath' and isinstance(val, dict) and 'value' in val:
                out.append(val['value'])
            else:
                collect_paths(val, out)
    elif isinstance(node, list):
        for val in node:
            collect_paths(val, out)


def check(path, pubs):
    layout = json.load(open(path))
    problems = []

    paths = []
    collect_paths(layout, paths)
    for p in sorted(set(paths)):
        topic, _, field = p.partition('.')
        if topic not in pubs:
            problems.append('%s: no publisher for %s' % (p, topic))
            continue
        msg = pubs[topic]
        if field and msg in FIELDS and field not in FIELDS[msg]:
            problems.append("%s: %s has no field '%s'" % (p, msg, field))

    for panel, cfg in layout['configById'].items():
        for topic in cfg.get('topics', {}):
            if topic not in pubs:
                problems.append('%s: no publisher for %s' % (panel, topic))
        for key in ('imageTopic', 'calibrationTopic'):
            topic = cfg.get('imageMode', {}).get(key)
            if topic and topic not in pubs:
                problems.append('%s: no publisher for %s (%s)' % (panel, topic, key))

    # Every Plot panel pins its Y range: auto-scale magnifies a converged signal into
    # noise and makes two runs incomparable.
    for pid, cfg in layout['configById'].items():
        if not pid.startswith('Plot!'):
            continue
        axes = [('minYValue', 'maxYValue')]
        if cfg.get('xAxisVal') == 'custom':
            axes.append(('minXValue', 'maxXValue'))
        for lo_key, hi_key in axes:
            lo, hi = cfg.get(lo_key), cfg.get(hi_key)
            if lo is None or hi is None:
                problems.append('%s: missing %s/%s (would auto-scale)' % (pid, lo_key, hi_key))
            elif lo >= hi:
                problems.append('%s: %s (%g) must be < %s (%g)' % (pid, lo_key, lo, hi_key, hi))

    # A thesis figure number must name exactly one panel. Two panels both titled "5.14b"
    # is how a plot ends up captioned as something it is not.
    figures = {}
    for pid, cfg in layout['configById'].items():
        m = re.match(r'\s*(\d+\.\d+[a-z]?)\b', cfg.get('title', ''))
        if m:
            figures.setdefault(m.group(1), []).append(cfg['title'])
    for fig, titles in sorted(figures.items()):
        if len(titles) > 1:
            problems.append('figure %s used by %d panels: %s' % (fig, len(titles), titles))

    # Every panel in the mosaic is configured, and every configured panel is placed.
    refs = set()

    # The root is a mosaic, not necessarily a single Tab panel: anything that must keep
    # streaming while you are looking elsewhere has to sit OUTSIDE the tab group, because
    # Foxglove unmounts inactive tabs and drops their subscriptions.
    tab_ids = set()

    def walk(node):
        if isinstance(node, str):
            refs.add(node)
            cfg = layout['configById'].get(node, {})
            if node.startswith('Tab!'):
                tab_ids.add(node)
                for tab in cfg.get('tabs', []):
                    walk(tab['layout'])
        elif isinstance(node, dict):
            walk(node.get('first'))
            walk(node.get('second'))

    walk(layout['layout'])
    # Tab panels are containers, not content: they are placed and configured, but comparing
    # them either way just adds noise.
    refs -= tab_ids
    declared = set(layout['configById']) - tab_ids
    if refs - declared:
        problems.append('placed but not configured: %s' % sorted(refs - declared))
    if declared - refs:
        problems.append('configured but never placed: %s' % sorted(declared - refs))

    return len(set(paths)), len(declared), problems


def main():
    pubs = publishers()
    failed = False
    # Every variant is checked, not just the canonical: a derived file with a bad image topic
    # draws nothing and looks exactly like a camera that is not publishing.
    for name in [os.path.basename(LAYOUT)] + list(variants.DERIVED):
        n_paths, n_panels, problems = check(os.path.join(HERE, name), pubs)
        print('%-22s %d message paths, %d panels' % (name, n_paths, n_panels))
        for line in problems:
            print('  ' + line)
        failed = failed or bool(problems)

    # The derived files are a copy of the canonical with one field replaced. If they have
    # drifted, every panel you fixed in the canonical is still broken in the other variant.
    drift = variants.stale()
    for line in drift:
        print('  ' + line)
    failed = failed or bool(drift)

    if failed:
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
