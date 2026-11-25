# syntax=docker/dockerfile:1.7
# fxteso_ibvs Raspberry Pi 5 image (linux/arm64): ROS 2 Jazzy + OpenCV 4.14 (NEON) + the Pi libcamera fork.
# Same base, snapshots and OpenCV as docker/Dockerfile; no Gazebo, no PX4 toolchain. Build: docker/dev.sh pi-build

ARG UBUNTU_IMAGE=ubuntu:24.04@sha256:534baea6a22c03a63003dbc8dbe78fe34bc0d7e595d9a9dc9834884ff530eb55
ARG UV_IMAGE=ghcr.io/astral-sh/uv:0.10.2@sha256:94a23af2d50e97b87b522d3cea24aaf8a1faedec1344c952767434f69585cbf9

FROM ${UV_IMAGE} AS uv

# ---------------------------------------------------------------------------------------------
FROM ${UBUNTU_IMAGE} AS base
ARG UBUNTU_SNAPSHOT=20261016T000000Z
ARG ROS_SNAPSHOT=2026-06-18
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Etc/UTC
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# APT::Snapshot maps ports.ubuntu.com to an amd64-only URL, so name the snapshot (it carries arm64) directly.
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
 && printf 'Types: deb\nURIs: https://snapshot.ubuntu.com/ubuntu/%s/\nSuites: noble noble-updates noble-backports noble-security\nComponents: main universe restricted multiverse\nSigned-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg\n' \
      "${UBUNTU_SNAPSHOT}" > /etc/apt/sources.list.d/ubuntu.sources \
 && rm -rf /var/lib/apt/lists/*

COPY keys/ros-snapshots.asc /usr/share/keyrings/ros-snapshots.asc
RUN echo "deb [signed-by=/usr/share/keyrings/ros-snapshots.asc] http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list

# ROS 2 Jazzy base: what the Pi runs (no desktop, no simulator).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-geometry \
      ros-jazzy-python-cmake-module \
      ros-jazzy-ament-cmake-ros \
      ros-dev-tools \
      python3-colcon-common-extensions \
      python3-colcon-override-check \
 && rm -rf /var/lib/apt/lists/*

# Build tools, the camera stack's build deps (tools/setup_pi5_ubuntu_camera.sh) and OpenCV's.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build meson git curl wget pkg-config sudo less file zstd \
      python3-dev python3-venv python3-ply python3-yaml python3-jinja2 pybind11-dev \
      libboost-dev libboost-program-options-dev libgnutls28-dev libssl-dev openssl \
      libglib2.0-dev libgstreamer-plugins-base1.0-dev libexif-dev libdrm-dev libcap-dev \
      libjpeg-dev libpng-dev libtiff-dev libwebp-dev libopenjp2-7-dev \
      libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
      libtbb-dev qtbase5-dev libeigen3-dev \
      v4l-utils i2c-tools udev \
 && rm -rf /var/lib/apt/lists/*

# Python from apt, as on the Pi; uv adds only what apt lacks (docker/pi/uv.lock).
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-numpy python3-scipy python3-yaml python3-psutil python3-serial python3-pil \
      python3-av python3-prctl python3-piexif python3-simplejson python3-jsonschema \
      python3-opencv \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------------------------
# OpenCV 4.14.0 + contrib, the RUNNING.md recipe with the Pi 5 (Cortex-A76) flags, into /usr/local.
FROM base AS opencv
ARG OPENCV_TAG=4.14.0
ARG OPENCV_COMMIT=0654a42e19215ef25b1d367d822f3c630447e7c7
ARG OPENCV_CONTRIB_COMMIT=a8e9acd62cabd30419dba83007f2ac0d07de5e2c
WORKDIR /src
RUN git clone --branch ${OPENCV_TAG} --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/opencv/opencv.git \
 && test "$(git -C opencv rev-parse HEAD)" = "${OPENCV_COMMIT}" \
 && git clone --branch ${OPENCV_TAG} --depth 1 https://github.com/opencv/opencv_contrib.git \
 && test "$(git -C opencv_contrib rev-parse HEAD)" = "${OPENCV_CONTRIB_COMMIT}"
RUN cmake -S opencv -B build -G Ninja \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D CMAKE_INSTALL_LIBDIR=lib/$(gcc -print-multiarch) \
      -D OPENCV_EXTRA_MODULES_PATH=/src/opencv_contrib/modules \
      -D WITH_TBB=ON \
      -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF \
      -D BUILD_opencv_python2=OFF -D BUILD_opencv_python3=OFF \
      -D CPU_BASELINE=NEON,FP16 -D CPU_DISPATCH=NEON_DOTPROD,NEON_FP16,NEON_BF16 \
      -D WITH_QT=5 -D WITH_GTK=OFF \
      2>&1 | tee /src/opencv-configure.log \
 && cmake --build build -j"$(nproc)" \
 && DESTDIR=/out cmake --install build \
 && cp /src/opencv-configure.log /out/usr/local/share/opencv4/configure.log

# ---------------------------------------------------------------------------------------------
# Pi 5 camera stack: libpisp first (or meson drops the pisp pipeline), then the libcamera fork, then kmsxx.
FROM base AS camera
ARG LIBPISP_TAG=v1.7.0
# The last release before libcamera moved to std::span (C++20); the workspace builds as C++17.
ARG LIBCAMERA_TAG=v0.7.2+rpt20260817
ARG KMSXX_COMMIT=4d2008f5893a5d6ea2b2bf5e70bc539bfcaf6db0
ARG PYINSTALL=/usr/local/lib/python3.12/dist-packages
WORKDIR /src
RUN git clone --branch ${LIBPISP_TAG} --depth 1 https://github.com/raspberrypi/libpisp.git \
 && meson setup libpisp/build libpisp --buildtype=release --prefix=/usr/local \
 && meson compile -C libpisp/build && meson install -C libpisp/build \
 && DESTDIR=/out meson install -C libpisp/build && ldconfig
ENV PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig:/usr/local/lib/pkgconfig
RUN git clone --branch ${LIBCAMERA_TAG} --depth 1 https://github.com/raspberrypi/libcamera.git \
 && meson setup libcamera/build libcamera --buildtype=release --prefix=/usr/local \
      -Dpython.platlibdir=${PYINSTALL} -Dpython.purelibdir=${PYINSTALL} \
      -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
      -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled \
      -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled \
 && ninja -C libcamera/build && DESTDIR=/out ninja -C libcamera/build install \
 && test -n "$(find /out/usr/local -name ipa_rpi_pisp.so)"
RUN git clone https://github.com/tomba/kmsxx.git && git -C kmsxx checkout ${KMSXX_COMMIT} \
 && meson setup kmsxx/build kmsxx --buildtype=release --prefix=/usr/local \
      -Dpython.platlibdir=${PYINSTALL} -Dpython.purelibdir=${PYINSTALL} \
 && ninja -C kmsxx/build && DESTDIR=/out ninja -C kmsxx/build install \
 && mv /out/usr/local/lib/*/python3*/site-packages/pykms /out${PYINSTALL}/ \
 && test -f /out${PYINSTALL}/pykms/pykms.so
# kmsxx ignores the python.* options and installs pykms off sys.path; it is moved beside libcamera.

# ---------------------------------------------------------------------------------------------
# MicroXRCEAgent at the workspace's submodule commit (v3.0.1); hardware.launch runs it from PATH.
FROM base AS agent
ARG XRCE_COMMIT=155cfaaf8b7abac2e85d4a62d3649b09ace0be55
# Its superbuild tracks the moving 3.x / 2.2.x branches; pinned as in tools/build_xrce_agent.sh.
ARG FASTDDS_COMMIT=980f1926194c54a4ae9f2056bc9503fae95fc4ac
ARG FASTCDR_COMMIT=757d5e422253568c487ad97c22e44680fc1ddbaf
WORKDIR /src
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git agent \
 && git -C agent checkout ${XRCE_COMMIT} \
 && sed -i -e "s/set(_fastdds_tag 3.x)/set(_fastdds_tag ${FASTDDS_COMMIT})/" \
           -e "s/set(_fastcdr_tag 2.2.x)/set(_fastcdr_tag ${FASTCDR_COMMIT})/" agent/CMakeLists.txt \
 && grep -q "${FASTDDS_COMMIT}" agent/CMakeLists.txt && grep -q "${FASTCDR_COMMIT}" agent/CMakeLists.txt \
 && cmake -S agent -B agent/build -D CMAKE_INSTALL_PREFIX=/usr/local \
 && cmake --build agent/build -j"$(nproc)" \
 && DESTDIR=/out cmake --install agent/build

# ---------------------------------------------------------------------------------------------
FROM base AS pi
ARG USERNAME=fxteso
ARG USER_UID=1000
ARG USER_GID=1000
ARG USER_HOME=/home/fxteso

COPY --from=opencv /out/usr/local/ /usr/local/
COPY --from=camera /out/usr/local/ /usr/local/
COPY --from=agent /out/usr/local/ /usr/local/
RUN ldconfig

# Workspace build deps not needed by the stages above (rosdep: cv_bridge's Python module).
RUN apt-get update && apt-get install -y --no-install-recommends libboost-python-dev \
 && rm -rf /var/lib/apt/lists/*

COPY --from=uv /uv /uvx /usr/local/bin/
COPY pi/pyproject.toml pi/uv.lock /opt/fxteso-env/
ENV UV_PROJECT_ENVIRONMENT=/opt/venv UV_PYTHON=/usr/bin/python3 UV_PYTHON_DOWNLOADS=never UV_LINK_MODE=copy
RUN uv venv --system-site-packages /opt/venv \
 && cd /opt/fxteso-env && uv sync --frozen --no-install-project --no-cache
ENV PATH=/opt/venv/bin:${PATH} VIRTUAL_ENV=/opt/venv PYTHONNOUSERSITE=1
RUN python3 -c "import libcamera, pykms, picamera2, cv2; print('camera stack ok, cv2', cv2.__version__)"

RUN userdel -r ubuntu 2>/dev/null || true \
 && groupadd -g ${USER_GID} ${USERNAME} \
 && useradd -m -d ${USER_HOME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash \
      -G dialout,plugdev,video,sudo ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

COPY entrypoint.sh /usr/local/bin/fxteso-entrypoint
COPY bashrc /etc/fxteso.bashrc
RUN echo 'source /etc/fxteso.bashrc' >> /etc/bash.bashrc
COPY manifest.sh /usr/local/bin/fxteso-manifest

USER ${USERNAME}
ENTRYPOINT ["/usr/local/bin/fxteso-entrypoint"]
CMD ["sleep", "infinity"]
