#!/usr/bin/env bash
# Build the Raspberry Pi camera stack on Ubuntu 24.04 (noble) / Raspberry Pi 5.
#
#     bash tools/setup_pi5_ubuntu_camera.sh                # libpisp + libcamera + picamera2
#     bash tools/setup_pi5_ubuntu_camera.sh --with-apps    # ... and rpicam-apps
#     bash tools/setup_pi5_ubuntu_camera.sh --skip-libcamera   # resume at the Python steps
#     bash tools/setup_pi5_ubuntu_camera.sh --jobs 2       # if the Pi runs out of RAM
#
# Why this exists. Ubuntu noble/universe ships UPSTREAM libcamera 0.2.0, and its
# libcamera-ipa package contains ipa_rkisp1.so, ipa_rpi_vc4.so and ipa_vimc.so - there is
# no ipa_rpi_pisp.so. The Pi 5's ISP is PiSP, so the stock packages cannot drive any camera
# on a Pi 5, and neither python3-picamera2 nor rpicam-apps exists in the archive at all.
#
# THE ORDER MATTERS. libcamera's rpi/pisp pipeline handler and IPA include libpisp headers
# and resolve it as pkg-config `libpisp`. Pi OS has libpisp-dev; Ubuntu has nothing, so
# libpisp must be built and installed FIRST. If it is missing, meson does not stop - it
# configures without the pisp pipeline, the build succeeds, the pisp tuning JSONs still get
# installed, and you end up with a complete-looking install whose only missing file is
# ipa_rpi_pisp.so. That failure is the reason this script grew a verification step.
#
# Expect 20-45 minutes on a Pi 5, most of it compiling libcamera.
set -euo pipefail

JOBS="$(nproc)"
WITH_APPS=0
SKIP_LIBCAMERA=0
SRC="${HOME}/src"

while [ $# -gt 0 ]; do
    case "$1" in
        --with-apps) WITH_APPS=1; shift ;;
        --skip-libcamera) SKIP_LIBCAMERA=1; shift ;;
        --jobs) JOBS="$2"; shift 2 ;;
        --src) SRC="$2"; shift 2 ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

say() { printf '\n=== %s\n' "$*"; }
ARCH="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || echo aarch64-linux-gnu)"
# meson's default python install dir is /usr/local/lib/python3/dist-packages - bare "python3",
# a path on nobody's sys.path. Both source-built Python modules below are pointed here instead.
PYMINOR="$(python3 -c 'import sys; print(sys.version_info[1])')"
PYINSTALL="/usr/local/lib/python3.${PYMINOR}/dist-packages"
# A source install lands in /usr/local; pkg-config does not search the multiarch directory
# there by default, which is exactly how libcamera fails to find the libpisp just installed.
export PKG_CONFIG_PATH="/usr/local/lib/${ARCH}/pkgconfig:/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

# --- sanity ------------------------------------------------------------------------------
MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
say "board: ${MODEL}"
case "${MODEL}" in
    *"Raspberry Pi 5"*) ;;
    *) echo "WARNING: written for the Pi 5 (PiSP). On a Pi 4 the vc4 pipeline in Ubuntu's"
       echo "         own package may already be enough." ;;
esac
if ! uname -r | grep -q -- '-raspi'; then
    echo "WARNING: kernel $(uname -r) is not the Raspberry Pi flavour. The IMX708 and PiSP"
    echo "         drivers live in linux-raspi:  sudo apt install linux-raspi && reboot"
fi

mkdir -p "${SRC}"

if [ "${SKIP_LIBCAMERA}" = "0" ]; then
    # --- 1. build dependencies -----------------------------------------------------------
    say "installing build dependencies"
    sudo apt update
    sudo apt install -y \
        git build-essential pkg-config meson ninja-build cmake \
        python3-dev python3-pip python3-venv python3-ply python3-yaml python3-jinja2 \
        pybind11-dev libboost-dev libgnutls28-dev libssl-dev openssl libtiff-dev \
        libglib2.0-dev libgstreamer-plugins-base1.0-dev libboost-program-options-dev \
        libexif-dev libavcodec-dev libdrm-dev libjpeg-dev libpng-dev \
        v4l-utils i2c-tools

    # --- 2. libpisp - MUST precede libcamera ---------------------------------------------
    say "building libpisp (the Pi 5 ISP configuration helper)"
    if [ -d "${SRC}/libpisp/.git" ]; then
        git -C "${SRC}/libpisp" pull --ff-only
    else
        git clone --depth 1 https://github.com/raspberrypi/libpisp.git "${SRC}/libpisp"
    fi
    cd "${SRC}/libpisp"
    rm -rf build
    meson setup build --buildtype=release --prefix=/usr/local
    meson compile -C build -j "${JOBS}"
    sudo meson install -C build
    sudo ldconfig
    if ! pkg-config --exists libpisp; then
        echo "ERROR: libpisp installed but pkg-config cannot see it." >&2
        echo "       PKG_CONFIG_PATH=${PKG_CONFIG_PATH}" >&2
        exit 1
    fi
    echo "libpisp $(pkg-config --modversion libpisp) visible to pkg-config"

    # --- 3. Raspberry Pi's libcamera fork ------------------------------------------------
    say "building libcamera (Raspberry Pi fork) - this is the long one"
    if [ -d "${SRC}/libcamera/.git" ]; then
        git -C "${SRC}/libcamera" pull --ff-only
    else
        git clone --depth 1 https://github.com/raspberrypi/libcamera.git "${SRC}/libcamera"
    fi
    cd "${SRC}/libcamera"
    rm -rf build
    # -Dpipelines/-Dipas are the whole point: rpi/pisp is the Pi 5 ISP, rpi/vc4 the Pi 4 one.
    # -Dpycamera=enabled builds the Python bindings picamera2 imports as `libcamera`.
    meson setup build --buildtype=release --prefix=/usr/local \
        -Dpython.platlibdir="${PYINSTALL}" \
        -Dpython.purelibdir="${PYINSTALL}" \
        -Dpipelines=rpi/vc4,rpi/pisp \
        -Dipas=rpi/vc4,rpi/pisp \
        -Dv4l2=enabled \
        -Dgstreamer=enabled \
        -Dtest=false \
        -Dlc-compliance=disabled \
        -Dcam=disabled \
        -Dqcam=disabled \
        -Ddocumentation=disabled \
        -Dpycamera=enabled
    ninja -C build -j "${JOBS}"
    sudo ninja -C build install
    sudo ldconfig

    # --- 4. verify the one file that decides everything ----------------------------------
    # Without this check the script reports success on an install that cannot open a camera.
    say "verifying the PiSP IPA module"
    # -maxdepth 5, not 4: libcamera 0.7 nests these under <libdir>/libcamera/ipa/.
    IPA_SO="$(find /usr/local/lib /usr/lib -maxdepth 5 -name 'ipa_rpi_pisp.so' 2>/dev/null \
              | head -1 || true)"
    if [ -z "${IPA_SO}" ]; then
        echo "ERROR: ipa_rpi_pisp.so was not installed - the pisp pipeline was configured" >&2
        echo "       out, almost always because libpisp was not found. The meson log says:" >&2
        grep -iE 'libpisp|pisp' "${SRC}/libcamera/build/meson-logs/meson-log.txt" \
            | tail -20 >&2 || true
        exit 1
    fi
    echo "found ${IPA_SO}"
fi

# --- 5. put the Python bindings on sys.path ----------------------------------------------
# meson installs them under a prefix-relative site-packages; Ubuntu's interpreter searches
# dist-packages. Without a bridge the bindings are installed and still not importable, which
# is the most confusing failure in this whole setup. Locate them rather than assume a path.
say "linking the libcamera Python bindings into Ubuntu's sys.path"
if ! python3 -c 'import libcamera' 2>/dev/null; then
    BIND="$(find /usr/local/lib /usr/lib -maxdepth 4 -name '_libcamera*.so' 2>/dev/null \
            | head -1 || true)"
    if [ -n "${BIND}" ]; then
        PKGROOT="$(dirname "$(dirname "${BIND}")")"
        PYVER="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
        DIST="/usr/local/lib/python${PYVER}/dist-packages"
        sudo mkdir -p "${DIST}"
        echo "${PKGROOT}" | sudo tee "${DIST}/libcamera-local.pth" >/dev/null
        echo "wrote ${DIST}/libcamera-local.pth -> ${PKGROOT}"
    else
        echo "WARNING: no _libcamera*.so found. The bindings were not built - check that"
        echo "         pybind11-dev was installed and that meson reported pycamera enabled:"
        echo "         grep -i pycamera ${SRC}/libcamera/build/meson-logs/meson-log.txt"
    fi
fi
# Deliberately not fatal: picamera2 below is still worth installing, and the preflight at
# the end reports this far better than an abort would.
python3 -c 'import libcamera; print("libcamera bindings:", libcamera.__file__)' \
    || echo "WARNING: libcamera bindings still not importable - see the preflight below"

# --- 6. rpicam-apps (optional) -----------------------------------------------------------
if [ "${WITH_APPS}" = "1" ]; then
    say "building rpicam-apps (gives rpicam-hello / rpicam-still)"
    if [ -d "${SRC}/rpicam-apps/.git" ]; then
        git -C "${SRC}/rpicam-apps" pull --ff-only
    else
        git clone --depth 1 https://github.com/raspberrypi/rpicam-apps.git "${SRC}/rpicam-apps"
    fi
    cd "${SRC}/rpicam-apps"
    rm -rf build
    meson setup build --buildtype=release --prefix=/usr/local
    ninja -C build -j "${JOBS}"
    sudo ninja -C build install
    sudo ldconfig
fi

# --- 7. pykms (kmsxx) --------------------------------------------------------------------
# Not optional, despite sounding like a preview extra: picamera2/previews/__init__.py does an
# UNCONDITIONAL `from .drm_preview import DrmPreview`, and drm_preview imports pykms. So
# `import picamera2` fails outright without it - headless, over ssh, with no DISPLAY, always.
# Pi OS gets this from python3-kms++; Ubuntu has no such package under any name, so it is a
# source build. Small and quick compared to libcamera.
say "building kmsxx for the pykms module"
if ! command -v meson >/dev/null; then
    sudo apt install -y meson ninja-build libdrm-dev pkg-config python3-dev pybind11-dev
fi
if [ -d "${SRC}/kmsxx/.git" ]; then
    git -C "${SRC}/kmsxx" pull --ff-only
else
    git clone --depth 1 https://github.com/tomba/kmsxx.git "${SRC}/kmsxx"
fi
cd "${SRC}/kmsxx"
rm -rf build
# pykms is built by default; naming the option risks the bool-vs-feature spelling changing
# between versions, so it is left alone and only the install location is forced.
meson setup build --buildtype=release --prefix=/usr/local \
    -Dpython.platlibdir="${PYINSTALL}" \
    -Dpython.purelibdir="${PYINSTALL}"
ninja -C build -j "${JOBS}"
sudo ninja -C build install
sudo ldconfig
python3 -c 'import pykms; print("pykms:", pykms.__file__)' \
    || echo "WARNING: pykms still not importable - picamera2 will not import either"

# --- 8. picamera2 ------------------------------------------------------------------------
# No python3-picamera2 in noble, so pip is the only option, and noble's PEP 668 marks the
# system interpreter externally managed. --break-system-packages is deliberate: picamera2
# must share an interpreter with the locally built bindings, which a plain venv cannot see.
say "installing picamera2 from pip"
# python3-prctl matters most here: picamera2 depends on python-prctl, which is a C extension
# whose sdist build fails with "You need to install libcap development headers". Ubuntu has it
# prebuilt, so apt sidesteps the build; libcap-dev is kept anyway for any pip fallback.
# pyqt5/opengl are what picamera2's QtGl preview needs - python3-kms++ is NOT packaged for
# Ubuntu, so the DRM (console) preview cannot work here and --preview needs a desktop session.
sudo apt install -y libcap-dev python3-prctl python3-piexif python3-pyqt5 python3-opengl \
    python3-numpy python3-pil python3-av python3-simplejson || true
pip install --break-system-packages picamera2 \
    || echo "WARNING: pip install picamera2 failed - see the preflight below"

# --- 9. firmware and permissions ---------------------------------------------------------
CFG=/boot/firmware/config.txt
if [ -f "${CFG}" ] && ! grep -qE '^(camera_auto_detect=1|dtoverlay=imx708)' "${CFG}"; then
    say "enabling the camera in ${CFG}"
    echo "camera_auto_detect=1" | sudo tee -a "${CFG}" >/dev/null
    echo "added camera_auto_detect=1 - REBOOT before the sensor appears"
fi
if ! id -nG "$USER" | tr ' ' '\n' | grep -qx video; then
    sudo usermod -aG video "$USER"
    echo "added $USER to the video group - log out and back in"
fi

# --- 10. hand back to the preflight -------------------------------------------------------
say "done - running the preflight"
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "${HERE}/camera_module3_wide_test.py" --doctor || true
cat <<'EOF'

If the preflight still reports blocking problems, the usual order is:
  1. reboot (config.txt and group changes need one)
  2. dmesg | grep -i imx708      - proves the kernel bound the sensor
  3. rpicam-hello --list-cameras - proves libcamera can open it   (needs --with-apps)
  4. python3 camera_module3_wide_test.py   - the real test

One consequence worth knowing for this workspace: ros-jazzy-camera-ros from apt links
against Ubuntu's libcamera 0.2.0, the one without PiSP. If you use it as the driver behind
hardware.launch.py's camera_topic, build it from source in the ROS workspace so it links
against the fork installed here, or it will fail the same way for the same reason.
EOF
