#!/usr/bin/env bash
# Kill a previous run and wait for the machine to be ready for the next one.
# The wait matters: orphaned gz servers and unreleased DDS ports break the next run's start-up.
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Every installed executable, derived rather than hand-listed (find -L: symlink install).
mapfile -t PATTERNS < <(find -L "$REPO"/install/*/lib -mindepth 2 -maxdepth 2 \
                             -type f -executable -printf '%f\n' 2>/dev/null | sort -u)

if [ "${#PATTERNS[@]}" -eq 0 ]; then
    echo "cleanup.sh: no node executables under $REPO/install - build the workspace first" >&2
    exit 1
fi

# ros2cli.daemon accumulates stale Fast-DDS segments; killed, since a wedged one ignores `stop`.
PATTERNS+=("gz sim" "px4_sitl" "MicroXRCEAgent" "parameter_bridge" "ros2cli.daemon")

# pgrep -f matches any command line CONTAINING the pattern, including our own and that of whatever
# shell invoked us - a caller that merely mentions "gz sim" used to get killed by its own cleanup.
self_tree() {
    local pid=$$
    while [ "${pid:-0}" -gt 1 ]; do
        echo "$pid"
        pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')"
    done
}

survivors() {
    local p
    for p in "${PATTERNS[@]}"; do pgrep -f "$p" 2>/dev/null; done \
        | grep -vxF -f <(self_tree) | sort -un
}

kill_all() {
    local sig="${1:-}" pids
    pids="$(survivors)"
    [ -n "$pids" ] && kill $sig $pids 2>/dev/null
    return 0
}

kill_all
sleep 3
kill_all -9

# Verify rather than trust the sleeps: a survivor is the one failure mode this script exists to
# prevent, and it is invisible in the next run's log.
for _ in $(seq 20); do
    [ -z "$(survivors)" ] && break
    sleep 1
done

if [ -n "$(survivors)" ]; then
    echo "cleanup.sh: these processes would not die - kill them by hand before relaunching:" >&2
    ps -o pid,lstart,cmd -p "$(survivors | tr '\n' ',' | sed 's/,$//')" >&2
    exit 1
fi

# Delete only the segments no live process has mapped.
mapped="$(cat /proc/[0-9]*/maps 2>/dev/null | grep -o '/dev/shm/[^ ]*' | sort -u)"
for f in /dev/shm/fastrtps_*; do
    [ -e "$f" ] || continue
    grep -qxF "$f" <<<"$mapped" || rm -f "$f"
done

sleep 12
