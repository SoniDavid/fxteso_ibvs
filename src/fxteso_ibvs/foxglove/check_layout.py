#!/usr/bin/env python3
"""Check fxteso_ibvs.json against the topics the package actually publishes.

A message path pointing at something that does not exist draws nothing rather than
erroring, which looks identical to a genuinely flat signal. Run after editing the layout:

    python3 src/fxteso_ibvs/foxglove/check_layout.py
"""
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = os.path.join(HERE, 'fxteso_ibvs.json')
SRC = os.path.join(HERE, os.pardir, 'src', '*.cpp')

# Only these are addressed by the layout; enough to catch a mistyped field name.
FIELDS = {
    'geometry_msgs::msg::Vector3': {'x', 'y', 'z'},
    'geometry_msgs::msg::Quaternion': {'x', 'y', 'z', 'w'},
    'std_msgs::msg::Float64': {'data'},
    'geometry_msgs::msg::Twist': {'linear', 'angular'},
    'nav_msgs::msg::Path': {'header', 'poses'},
    'visualization_msgs::msg::MarkerArray': {'markers'},
}


def publishers():
    """topic -> message type, from every live create_publisher in the package."""
    found = {}
    for path in glob.glob(SRC):
        # Strip comments first, then match whole-file: some declarations wrap across lines.
        src = re.sub(r'//[^\n]*', '', open(path).read())
        for m in re.finditer(r'create_publisher<([^>]+)>\(\s*"([^"]+)"', src):
            found['/' + m.group(2).lstrip('/')] = m.group(1)
    # Bridged in from Gazebo rather than published by us; see config/bridge.yaml.
    found['/quad/camera/image_raw'] = 'sensor_msgs::msg::Image'
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


def main():
    layout = json.load(open(LAYOUT))
    pubs = publishers()
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
        image = cfg.get('imageMode', {}).get('imageTopic')
        if image and image not in pubs:
            problems.append('%s: no publisher for %s' % (panel, image))

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

    def mosaic(n):
        if isinstance(n, str):
            refs.add(n)
        elif isinstance(n, dict):
            mosaic(n.get('first'))
            mosaic(n.get('second'))

    root = layout['layout']
    for tab in layout['configById'][root]['tabs']:
        mosaic(tab['layout'])
    declared = set(layout['configById']) - {root}
    if refs - declared:
        problems.append('placed but not configured: %s' % sorted(refs - declared))
    if declared - refs:
        problems.append('configured but never placed: %s' % sorted(declared - refs))

    print('%d message paths, %d panels checked' % (len(set(paths)), len(declared)))
    if problems:
        print('\n'.join(problems))
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
