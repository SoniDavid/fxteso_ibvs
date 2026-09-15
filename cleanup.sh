#!/usr/bin/env bash
# Kill a previous run and wait for the machine to be ready for the next one.
#
# The wait is not optional. An orphaned gz server starves PX4's magnetometer, and the DDS graph
# and the simulator's UDP ports need time to be released - a run started seconds after a kill
# sits waiting on EKF2 for its whole timeout.
#
# ros2 launch signals only the `ruby .../gz sim` wrapper, and its `gz sim server` / `gz sim gui`
# children are forked, so they survive and reparent to systemd --user. They do match the patterns
# below (ruby's setproctitle rewrites argv, which is what pgrep -f reads) - but the old version
# never checked, so a kill that missed looked identical to one that worked.
#
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Derived, not hand-listed. The hand-written list silently omitted sim_pilot, tf_broadcaster,
# gz_pose_broadcaster, td_linear, td_attitude*, px4_offboard_bridge and px4_state_adapter, so every
# run that did not exit gracefully leaked 7 spinning nodes. Thirty runs reached load 303, the
# simulator fell to RTF 0.06, and runs then failed for reasons that looked like anything but this.
# find -L: colcon --symlink-install makes each of these a symlink into build/, not a regular file.
mapfile -t PATTERNS < <(find -L "$REPO"/install/*/lib -mindepth 2 -maxdepth 2 \
                             -type f -executable -printf '%f\n' 2>/dev/null | sort -u)

if [ "${#PATTERNS[@]}" -eq 0 ]; then
    echo "cleanup.sh: no node executables under $REPO/install - build the workspace first" >&2
    exit 1
fi

# ros2cli.daemon outlives every run and accumulates one Fast-DDS segment per participant it ever
# discovered - 28 after an afternoon, which is what every run's "Failed init_port fastrtps_port7006:
# open_and_lock_file failed" reports. It respawns on demand, and it is killed rather than stopped
# because once wedged `ros2 daemon stop` hangs and the daemon survives.
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
