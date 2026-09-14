#!/usr/bin/env bash
# Launch a PX4 SITL run, and relaunch it when the start-up sensor race loses a sensor.
#
#     tools/run.sh -- target_profile:=line target_speed:=0.7 venue:=indoor
#     tools/run.sh -n 1 -- venue:=indoor          # one attempt, for measuring the race
#
# A Gazebo sensor stream (imu, magnetometer or barometer) intermittently fails to reach PX4's
# gz_bridge: the subscribe succeeds, no message ever arrives, and arming is blocked for the whole
# run. gz_bridge re-subscribes three times before giving up, so most of those heal in ~2 s and
# never reach this script. This catches the rest.
#
# A successful flight returns on the first attempt, so this is the entry point for every run, not
# just flaky ones. Anything that is not the sensor race - a bad launch argument, an unbuilt PX4,
# a hover_thrust mismatch - exits immediately with the launch's own status.
#
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAX_ATTEMPTS=5

while [ $# -gt 0 ]; do
    case "$1" in
        -n) [ $# -ge 2 ] || { echo "run.sh: -n needs a value" >&2; exit 1; }
            MAX_ATTEMPTS="$2"; shift 2 ;;
        --) shift; break ;;
        -h|--help) sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *) break ;;
    esac
done

if ! [[ "$MAX_ATTEMPTS" =~ ^[1-9][0-9]*$ ]]; then
    echo "run.sh: -n wants a positive integer, got '$MAX_ATTEMPTS'" >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "run.sh: ros2 not on PATH - source /opt/ros/jazzy/setup.bash and this workspace first" >&2
    exit 1
fi

# Ctrl-C must stop the whole thing, not count as a failed attempt and relaunch. A terminal delivers
# it to the whole foreground group, but anything signalling this script alone (a supervisor, kill
# -INT) would leave the launch running and this script waiting on it forever - so forward it and
# let the launch tear down before returning.
INTERRUPTED=0
LAUNCH_PID=
on_interrupt() {
    INTERRUPTED=1
    [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
}
trap on_interrupt INT TERM

# Markers that mean "the transport lost a sensor, relaunching is the fix". Everything else is a
# real failure and must not be retried five times.
TRANSIENT=(
    'px4_takeoff_gate failed - estimators not started'
    'attempts - the gz publisher never reached PX4'
    'px4_sitl exited with'
)

lost_sensor() {
    # gz_bridge's own verdict first; PX4's preflight text is the fallback. Tested for emptiness,
    # not exit status - `grep | tail` succeeds on no matches at all.
    local found
    found="$(grep -oE 'no (imu|magnetometer|barometer|clock) data from [^ ]+' "$1" | tail -1)"
    [ -z "$found" ] && found="$(grep -oE 'Found 0 compass|barometer 0 missing|No valid data from (Accel|Gyro) 0' "$1" | sort -u | paste -sd',' - | tr ',' ' ')"
    echo "${found:-an unidentified sensor - see $1}"
}

stamp="$(date +%Y%m%d_%H%M%S)"
declare -a VERDICTS=()

for attempt in $(seq "$MAX_ATTEMPTS"); do
    log="$REPO/bags/run_${stamp}_a${attempt}.log"
    echo "run.sh: attempt $attempt/$MAX_ATTEMPTS -> $log"

    bash "$REPO/cleanup.sh" || exit 1

    ros2 launch quad_px4 sitl.launch.py "$@" > >(tee "$log") 2>&1 &
    LAUNCH_PID=$!
    # Polled, not `wait`: traps run between commands, so this reacts to Ctrl-C promptly and still
    # sees the launch all the way through its own shutdown. Once interrupted the wait is bounded -
    # ros2 launch has been seen shut all its nodes down and then never exit itself, which would
    # otherwise hang this script forever.
    grace=0
    while kill -0 "$LAUNCH_PID" 2>/dev/null; do
        sleep 1

        if [ "$INTERRUPTED" = 1 ]; then
            grace=$((grace + 1))
            [ "$grace" -eq 30 ] && kill -TERM "$LAUNCH_PID" 2>/dev/null
            [ "$grace" -ge 45 ] && { kill -KILL "$LAUNCH_PID" 2>/dev/null; break; }
        fi
    done
    wait "$LAUNCH_PID"
    status=$?
    LAUNCH_PID=

    if [ "$INTERRUPTED" = 1 ]; then
        echo
        echo "run.sh: interrupted - reaping the run so the next one starts clean"
        # Not optional: a run torn down harder than a clean shutdown leaves nodes spinning, and
        # those accumulate into a machine slow enough to fail runs on its own.
        bash "$REPO/cleanup.sh" || true
        exit 130
    fi

    # tee lives in a process substitution, which bash does not reap with the job - give it a
    # moment to flush or the greps below can read a truncated log.
    sleep 1

    hit=''
    for m in "${TRANSIENT[@]}"; do
        grep -qF "$m" "$log" && { hit="$m"; break; }
    done

    if [ -z "$hit" ]; then
        echo "run.sh: attempt $attempt finished (launch status $status), not the sensor race"
        exit "$status"
    fi

    VERDICTS+=("attempt $attempt: $(lost_sensor "$log")")
    echo "run.sh: lost ${VERDICTS[-1]#*: }"
done

echo "run.sh: gave up after $MAX_ATTEMPTS attempts:" >&2
printf '  %s\n' "${VERDICTS[@]}" >&2
echo "  The same sensor every time is a different problem from a different one each time." >&2
exit 1
