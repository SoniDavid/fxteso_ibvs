#!/usr/bin/env bash
# Build MicroXRCEAgent from external/ with its Fast-DDS/Fast-CDR pinned (its superbuild tracks moving branches).
#     tools/build_xrce_agent.sh [build_dir]      # default external/Micro-XRCE-DDS-Agent/build
set -euo pipefail
FASTDDS_COMMIT=980f1926194c54a4ae9f2056bc9503fae95fc4ac   # keep in sync with docker/pi/Dockerfile
FASTCDR_COMMIT=757d5e422253568c487ad97c22e44680fc1ddbaf
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/external/Micro-XRCE-DDS-Agent"
BUILD="${1:-$SRC/build}"

# A pinned copy of the source lives in the build dir, so the submodule stays clean and rebuilds stay pinned.
mkdir -p "$BUILD"
rsync -a --delete --exclude /build --exclude /.git "$SRC/" "$BUILD/agent-src/"
sed -i -e "s/set(_fastdds_tag 3.x)/set(_fastdds_tag ${FASTDDS_COMMIT})/" \
       -e "s/set(_fastcdr_tag 2.2.x)/set(_fastcdr_tag ${FASTCDR_COMMIT})/" "$BUILD/agent-src/CMakeLists.txt"
grep -q "$FASTDDS_COMMIT" "$BUILD/agent-src/CMakeLists.txt" && grep -q "$FASTCDR_COMMIT" "$BUILD/agent-src/CMakeLists.txt" \
  || { echo "pin failed: the agent's dependency tags changed" >&2; exit 1; }
cmake -S "$BUILD/agent-src" -B "$BUILD"
cmake --build "$BUILD" -j"$(nproc)"
echo "built $BUILD/MicroXRCEAgent (install: sudo cmake --install $BUILD && sudo ldconfig)"
