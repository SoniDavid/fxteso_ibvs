#!/usr/bin/env python3
"""Compare a container manifest against a host one: every container entry should match the host's."""
import sys
from collections import defaultdict

RELEVANT = ('ros-', 'python3-', 'gz-', 'libgz-', 'gcc-', 'g++-', 'gstreamer')


def load(path):
    table = defaultdict(dict)
    with open(path) as f:
        for line in f:
            parts = line.rstrip('\n').split('\t')
            if len(parts) == 3:
                table[parts[0]][parts[1]] = parts[2]
    return table


def main(host_path, container_path):
    host, cont = load(host_path), load(container_path)
    for section in ('dpkg', 'python', 'tool', 'pyimport', 'git'):
        same = differ = only = 0
        rows = []
        for name, version in sorted(cont[section].items()):
            hv = host[section].get(name)
            if hv is None:
                only += 1
                rows.append(f'  + {name:45s} {version}  (not on host)')
            elif hv != version:
                differ += 1
                rows.append(f'  ~ {name:45s} host {hv}  |  container {version}')
            else:
                same += 1
        # Host-only matters for build/runtime deps; the host's desktop packages would drown it otherwise.
        missing = [n for n in sorted(host[section]) if n not in cont[section]
                   and (section != 'dpkg' or n.startswith(RELEVANT) or n.endswith('-dev'))]
        rows += [f'  - {n:45s} {host[section][n]}  (host only)' for n in missing]
        print(f'[{section}] same {same}, different {differ}, container-only {only}, '
              f'host-only {len(missing)}')
        print('\n'.join(rows))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit('usage: manifest_compare.py HOST.tsv CONTAINER.tsv')
    main(sys.argv[1], sys.argv[2])
