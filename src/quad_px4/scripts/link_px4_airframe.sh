#!/usr/bin/env bash
# Symlinks this package's airframe into the PX4 tree so PX4 SITL can autostart it.
# PX4's airframes/CMakeLists.txt already lists 4100_gz_F450_px4, so a missing or stale link
# breaks the PX4 build outright - this is a prerequisite, not a convenience.
#
#   ./link_px4_airframe.sh [/path/to/PX4-Autopilot]

set -euo pipefail

PX4_DIR="${1:-${PX4_DIR:-$HOME/Robotics/PX4-Autopilot}}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/../px4/airframes/4100_gz_F450_px4"
DEST_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"

if [ ! -d "$DEST_DIR" ]; then
	echo "error: $DEST_DIR does not exist - is $PX4_DIR a PX4-Autopilot checkout?" >&2
	exit 1
fi

SRC="$(cd "$(dirname "$SRC")" && pwd)/$(basename "$SRC")"
ln -sfn "$SRC" "$DEST_DIR/4100_gz_F450_px4"
echo "linked $DEST_DIR/4100_gz_F450_px4 -> $SRC"

# A dangling link fails late and confusingly at PX4 build time; prove it resolves now.
if [ ! -r "$DEST_DIR/4100_gz_F450_px4" ]; then
	echo "error: the link does not resolve to a readable file" >&2
	exit 1
fi

if ! grep -q '4100_gz_F450_px4' "$DEST_DIR/CMakeLists.txt"; then
	echo "warning: 4100_gz_F450_px4 is not listed in $DEST_DIR/CMakeLists.txt;" >&2
	echo "         add it there or PX4 will not install the airframe." >&2
fi

echo "now rebuild PX4:  make -C $PX4_DIR px4_sitl_default"
