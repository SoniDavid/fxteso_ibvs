#!/bin/bash
# fxteso_ibvs containers. Desktop: build | up | shell | run CMD... | down | manifest | save. Pi: pi-*
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FXTESO_WS="$(cd "$HERE/.." && pwd)"
export FXTESO_EXP="${FXTESO_EXP:-$(dirname "$FXTESO_WS")/fxteso_ibvs_experiments}"
export FXTESO_USER="$(id -un)" FXTESO_UID="$(id -u)" FXTESO_GID="$(id -g)" FXTESO_HOME="$HOME"
export FXTESO_XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
export FXTESO_TZ="${TZ:-$(cat /etc/timezone 2>/dev/null || echo Etc/UTC)}"
export FXTESO_IMAGE_TAG="${FXTESO_IMAGE_TAG:-latest}"
compose() { docker compose -f "$HERE/compose.yaml" "$@"; }

prepare() {
  # Shadow-mount sources must exist (and be ours) before docker would create them as root.
  mkdir -p "$FXTESO_WS"/.docker/{build,install,log,px4-build,xrce-build} "$FXTESO_EXP"
  touch "$FXTESO_WS/.docker/COLCON_IGNORE"
  [ -e "$FXTESO_XAUTH" ] || touch "$FXTESO_XAUTH"
}

pi_compose() { docker compose -f "$HERE/pi/compose.yaml" "$@"; }

cmd="${1:-shell}"; shift || true
case "$cmd" in
  # Raspberry Pi image: build/save on the desktop (arm64 emulation), up/shell/run/down on the Pi.
  pi-build) docker buildx build --platform linux/arm64 --load -f "$HERE/pi/Dockerfile" \
              -t "fxteso-ibvs-pi:${FXTESO_IMAGE_TAG}" "$@" "$HERE" ;;
  pi-save)
    out="${1:-$FXTESO_WS/.docker/fxteso-ibvs-pi_${FXTESO_IMAGE_TAG}.tar.zst}"
    docker save "fxteso-ibvs-pi:${FXTESO_IMAGE_TAG}" | zstd -T0 -10 -o "$out"
    echo "saved $out  (on the Pi: zstd -dc <file> | docker load)" ;;
  pi-up)    mkdir -p "$FXTESO_WS"/.docker-pi/{build,install,log}; pi_compose up -d "$@" ;;
  pi-down)  pi_compose down "$@" ;;
  pi-shell) pi_compose exec pi bash -i ;;
  pi-run)   pi_compose exec -T pi bash -c 'source /etc/fxteso.bashrc; "$@"' _ "$@" ;;
  build)    prepare; compose build "$@" ;;
  up)       prepare; compose up -d "$@" ;;
  down)     compose down "$@" ;;
  shell)    compose exec dev bash -i ;;
  run)      compose exec -T dev bash -c 'source /etc/fxteso.bashrc; "$@"' _ "$@" ;;
  manifest) compose exec -T dev /usr/local/bin/fxteso-manifest ;;
  save)
    out="${1:-$FXTESO_WS/.docker/fxteso-ibvs-dev_${FXTESO_IMAGE_TAG}.tar.zst}"
    docker save "fxteso-ibvs-dev:${FXTESO_IMAGE_TAG}" | zstd -T0 -10 -o "$out"
    docker image inspect --format '{{.Id}}' "fxteso-ibvs-dev:${FXTESO_IMAGE_TAG}" > "$out.id"
    echo "saved $out" ;;
  *) echo "usage: $0 build|up|shell|run CMD...|down|manifest|save [file] | pi-build|pi-save|pi-up|pi-shell|pi-run|pi-down" >&2; exit 2 ;;
esac
