#!/usr/bin/env bash
# One-shot IBVS-stack real-time benchmark for the Pi.
#
#   src/quad_cam/scripts/bench.sh [--load synthetic|plate|none] [--att true|false]
#                                 [--duration SECONDS] [--camera PRESET] [--cv-threads N]
#                                 [--backend opencv|nano]
#
# Launches quad_utils/hardware.launch.py + probe_bench, then writes bench_<ts>.log with
# the per-node SimRate 'loop:' self-reports and probe_bench's rate / CPU / thermal table.
# Assumes ROS + the workspace overlay are already sourced.
set -u

LOAD=synthetic
ATT=false
DURATION=120
CAMERA=module3wide_2304
CVT=0
BACKEND=opencv
while [ $# -gt 0 ]; do
    case "$1" in
        --load) LOAD=$2; shift 2 ;;
        --att) ATT=$2; shift 2 ;;
        --duration) DURATION=$2; shift 2 ;;
        --camera) CAMERA=$2; shift 2 ;;
        --cv-threads) CVT=$2; shift 2 ;;
        --backend) BACKEND=$2; shift 2 ;;
        *) echo "unknown arg: $1"; exit 2 ;;
    esac
done

WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
TS=$(date +%Y%m%d_%H%M%S)
LAUNCH_LOG="$WS/bench_launch_$TS.log"
PROBE_LOG="$WS/bench_probe_$TS.log"
REPORT="$WS/bench_$TS.log"

command -v ros2 >/dev/null || { echo "source ROS + the workspace overlay first"; exit 1; }

echo ">> cleanup"
( cd "$WS" && ./cleanup.sh ) || true

echo ">> launch: hardware.launch.py load:=$LOAD attitude_controller:=$ATT camera:=$CAMERA cv_num_threads:=$CVT detector_backend:=$BACKEND"
ros2 launch quad_cam hardware.launch.py \
    load:="$LOAD" attitude_controller:="$ATT" camera:="$CAMERA" cv_num_threads:="$CVT" \
    detector_backend:="$BACKEND" \
    > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null; ( cd "$WS" && ./cleanup.sh ) >/dev/null 2>&1' EXIT

echo ">> warm-up (18 s: camera AE + node stagger)"
sleep 18

echo ">> probe_bench for ${DURATION}s"
# synthetic load remaps image_features' output aside; plate/none leave it on /ImFeat_valid
IMFEAT=/ImFeat_valid_probe
[ "$LOAD" = synthetic ] || IMFEAT=/ImFeat_valid
ros2 run quad_cam probe_bench --ros-args \
    -p duration_s:="$DURATION" -p imfeat_topic:="$IMFEAT" \
    2>&1 | tee "$PROBE_LOG"

kill $LAUNCH_PID 2>/dev/null
sleep 2

{
    echo "# IBVS stack benchmark  $TS"
    echo "# load=$LOAD  attitude_controller=$ATT  camera=$CAMERA  duration=${DURATION}s  cv_num_threads=$CVT  detector_backend=$BACKEND"
    echo "# git: $(git -C "$WS" rev-parse --short HEAD 2>/dev/null || echo n/a)"
    echo "# $(uname -srm)   model: $(tr -d '\0' < /proc/device-tree/model 2>/dev/null)"
    echo "# baseline: $(vcgencmd measure_temp 2>/dev/null)  $(vcgencmd get_throttled 2>/dev/null)"
    echo
    echo "=== SimRate self-reports (per node) ==="
    grep -E 'loop: target' "$LAUNCH_LOG" || echo "(none - check $LAUNCH_LOG)"
    echo
    echo "=== probe_bench ==="
    cat "$PROBE_LOG"
} > "$REPORT"

echo ">> wrote $REPORT"
