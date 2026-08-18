"""Shared bag reading for the analysis scripts.

All three take one or more bag directories and print a comparison, so the common work is
opening an mcap bag and pulling a few topics out of it in timestamp order.

A bag whose recorder was killed rather than stopped has no metadata.yaml; rosbag2 cannot
open it. Recover with:  ros2 bag reindex <bag> -s mcap
"""
import os

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

# image_features publishes exactly this when it has fewer than four markers. Literals, never
# arithmetic, so exact comparison is intended - same test ibvs_gate.cpp makes.
NO_LOCK = (0.0, 0.0, 1.0, 0.0)


def is_locked(msg):
    return (msg.x, msg.y, msg.z, msg.w) != NO_LOCK


def read(bag, topics):
    """Yield (t_seconds_from_start, topic, msg) for `topics`, in recorded order."""
    if not os.path.isdir(bag):
        raise SystemExit('no such bag: %s' % bag)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        raise SystemExit('%s does not contain %s - was it recorded with rosbag:=true?'
                         % (bag, ', '.join(missing)))
    wanted = set(topics)
    t0 = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in wanted:
            continue
        if t0 is None:
            t0 = stamp
        yield (stamp - t0) / 1e9, topic, deserialize_message(data, get_message(types[topic]))


def locked_window(bag, settle=3.0, need=40):
    """First sustained locked stretch, as (start, end) seconds, `settle` trimmed off the front.

    The start-up transient after the controllers arm is not steady behaviour and would
    dominate every statistic, hence the trim.
    """
    start, run, last = None, 0, 0.0
    for t, _, msg in read(bag, ['/ImFeat_vector']):
        last = t
        if is_locked(msg):
            run += 1
            if run == 1:
                start = t
            continue
        if start is not None and run >= need:
            return start + settle, t
        run, start = 0, None
    if start is None:
        raise SystemExit('%s never held a marker lock' % bag)
    return start + settle, last
