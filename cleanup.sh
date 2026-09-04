#!/usr/bin/env bash
# Kill a previous run and wait for the machine to be ready for the next one.
#
# The wait is not optional. An orphaned gz server starves PX4's magnetometer, and the DDS graph
# and the simulator's UDP ports need time to be released - a run started seconds after a kill
# sits waiting on EKF2 for its whole timeout.
#
for p in "[g]z sim" "[p]x4_sitl" "[M]icroXRCEAgent" "[p]arameter_bridge" "[c]amera_distort" \
         "[t]arget_position" "[d]isturbances" "[i]mage_features" "[p]os_ctrl" \
         "[f]ixed_eso" "[i]bvs_gate" "[p]x4_takeoff_gate"; do
    pkill -f "$p" 2>/dev/null
done
sleep 3
pkill -9 -f "[g]z sim" 2>/dev/null
pkill -9 -f "[p]x4_sitl" 2>/dev/null
pkill -9 -f "[M]icroXRCEAgent" 2>/dev/null
sleep 12
