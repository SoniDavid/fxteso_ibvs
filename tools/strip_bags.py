#!/usr/bin/env python3
"""Drop the cumulative Path topics from recorded bags. They are 80% of a bag and carry nothing
that /quad_position and /tgt_position do not already hold at 100 Hz - see config/bag_topics.yaml.

  python3 tools/strip_bags.py --dry-run              # report what would change
  python3 tools/strip_bags.py --pilot OUT bag...     # convert elsewhere, keep originals
  python3 tools/strip_bags.py                        # convert and replace, verified per bag

Idempotent and resumable: a bag with no Path topics is skipped. An original is removed only
after its replacement passes the per-topic count and duration check.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

import yaml

EXCLUDE = ['/quad_path', '/tgt_path', '/ibvs_markers']
BAGS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'bags')


def info(bag):
    """Per-topic message counts and duration, or None if the bag has no readable metadata."""
    path = os.path.join(bag, 'metadata.yaml')
    if not os.path.isfile(path):
        return None
    d = yaml.safe_load(open(path))['rosbag2_bagfile_information']
    counts = {t['topic_metadata']['name']: t['message_count']
              for t in d['topics_with_message_count']}
    return counts, d['duration']['nanoseconds'], d['message_count']


def reindex(bag):
    subprocess.run(['ros2', 'bag', 'reindex', bag, '-s', 'mcap'],
                   check=False, capture_output=True)


def convert(bag, dest):
    """ros2 bag convert into dest/<name>. Returns True on success."""
    name = os.path.basename(bag)
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as fh:
        yaml.safe_dump({'output_bags': [{'uri': os.path.join(dest, name),
                                         'storage_id': 'mcap',
                                         'all_topics': True,
                                         'exclude_topics': EXCLUDE}]}, fh)
        opts = fh.name
    try:
        r = subprocess.run(['ros2', 'bag', 'convert', '-i', bag, '-o', opts],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print('    convert failed: %s' % (r.stderr.strip().splitlines() or [''])[-1])
            return False
        return True
    finally:
        os.unlink(opts)


def verify(before, after):
    """Every retained topic must survive with an identical count, and the span must match."""
    bc, bdur, _ = before
    ac, adur, _ = after
    if adur != bdur:
        return 'duration %d != %d' % (adur, bdur)
    for topic, n in bc.items():
        if topic in EXCLUDE:
            if topic in ac:
                return '%s survived the exclude' % topic
            continue
        if ac.get(topic) != n:
            return '%s: %s != %d' % (topic, ac.get(topic), n)
    extra = set(ac) - set(bc)
    if extra:
        return 'unexpected topics %s' % sorted(extra)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--pilot', metavar='OUTDIR', help='convert into OUTDIR, keep the originals')
    ap.add_argument('bags', nargs='*', help='bag dirs (default: every bags/px4_*)')
    a = ap.parse_args()

    bags = a.bags or sorted(os.path.join(BAGS, d) for d in os.listdir(BAGS)
                            if d.startswith('px4_') and os.path.isdir(os.path.join(BAGS, d)))
    todo, saved, done, failed = [], 0, 0, []
    for b in bags:
        i = info(b)
        if i is None:
            print('%s: no metadata.yaml, reindexing' % os.path.basename(b))
            reindex(b)
            i = info(b)
            if i is None:
                failed.append((b, 'unreadable even after reindex'))
                continue
        if not any(t in i[0] for t in EXCLUDE):
            continue
        todo.append((b, i))

    print('%d of %d bags carry the viz topics' % (len(todo), len(bags)))
    if a.dry_run:
        for b, i in todo:
            print('  %-28s %6.1f MB' % (os.path.basename(b), du(b) / 2**20))
        return 0

    dest = a.pilot
    if dest:
        os.makedirs(dest, exist_ok=True)
    for n, (b, before) in enumerate(todo, 1):
        name = os.path.basename(b)
        out = dest or tempfile.mkdtemp(prefix='strip_', dir=os.path.dirname(BAGS))
        print('[%d/%d] %s' % (n, len(todo), name), flush=True)
        if not convert(b, out):
            failed.append((b, 'convert failed'))
            if not dest:
                shutil.rmtree(out, ignore_errors=True)
            continue
        new = os.path.join(out, name)
        why = verify(before, info(new))
        if why:
            print('    VERIFY FAILED (%s) - original kept' % why)
            failed.append((b, why))
            if not dest:
                shutil.rmtree(out, ignore_errors=True)
            continue
        if dest:
            done += 1
            continue
        old_sz, new_sz = du(b), du(new)
        shutil.rmtree(b)
        shutil.move(new, b)
        shutil.rmtree(out, ignore_errors=True)
        saved += old_sz - new_sz
        done += 1
        print('    %.0f -> %.0f MB' % (old_sz / 2**20, new_sz / 2**20))

    print('\n%d converted, %d failed, %.1f GB reclaimed' % (done, len(failed), saved / 2**30))
    for b, why in failed:
        print('  FAILED %s: %s' % (os.path.basename(b), why))
    return 1 if failed else 0


def du(path):
    return sum(os.path.getsize(os.path.join(r, f))
               for r, _, fs in os.walk(path) for f in fs)


if __name__ == '__main__':
    sys.exit(main())
