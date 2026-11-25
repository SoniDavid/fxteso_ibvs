# syntax=docker/dockerfile:1.7
# fxteso_ibvs dev image: Ubuntu 24.04 + ROS 2 Jazzy (dated snapshot) + vendor Gazebo 8.11 + OpenCV 4.14.
# Every source is pinned: base digest, Ubuntu snapshot, ROS snapshot, git tags, uv.lock.

ARG UBUNTU_IMAGE=ubuntu:24.04@sha256:534baea6a22c03a63003dbc8dbe78fe34bc0d7e595d9a9dc9834884ff530eb55
ARG UV_IMAGE=ghcr.io/astral-sh/uv:0.10.2@sha256:94a23af2d50e97b87b522d3cea24aaf8a1faedec1344c952767434f69585cbf9

FROM ${UV_IMAGE} AS uv

# ---------------------------------------------------------------------------------------------
FROM ${UBUNTU_IMAGE} AS base
ARG UBUNTU_SNAPSHOT=20261016T000000Z
ARG ROS_SNAPSHOT=2026-06-18
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Etc/UTC
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# snapshot.ubuntu.com is https-only, so ca-certificates comes from the live archive first.
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
 && echo "APT::Snapshot \"${UBUNTU_SNAPSHOT}\";" > /etc/apt/apt.conf.d/50snapshot \
 && rm -rf /var/lib/apt/lists/*

COPY keys/ros-snapshots.asc /usr/share/keyrings/ros-snapshots.asc
RUN echo "deb [signed-by=/usr/share/keyrings/ros-snapshots.asc] http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list

# ROS 2 Jazzy, with Gazebo Harmonic only through the ROS vendor packages (gz-sim 8.11).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      ros-jazzy-ros-gz \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-geometry \
      ros-jazzy-python-cmake-module \
      ros-jazzy-ament-cmake-ros \
      python3-colcon-common-extensions \
      python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

# Build tools, PX4 (Tools/setup/ubuntu.sh: base, NuttX, sim minus gz-harmonic) and the fmu-v6c toolchain.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build ccache git curl wget gnupg lsb-release sudo pkg-config \
      file rsync unzip zip zstd less bash-completion gdb \
      python3-dev python3-venv python3-setuptools python3-wheel \
      astyle cppcheck lcov libssl-dev libxml2-dev libxml2-utils shellcheck \
      automake binutils-dev bison flex gdb-multiarch genromfs gettext gperf kconfig-frontends \
      libelf-dev libexpat-dev libgmp-dev libisl-dev libmpc-dev libmpfr-dev libncurses-dev \
      libtool libunwind-dev texinfo u-boot-tools util-linux vim-common \
      gcc-multilib g++-multilib \
      gcc-arm-none-eabi=15:13.2.rel1-2 \
      libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
      bc dmidecode cppzmq-dev protobuf-compiler libprotobuf-dev libzmq3-dev uuid-dev libsqlite3-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer-plugins-base1.0-dev \
      libeigen3-dev libimage-exiftool-perl libopencv-dev python3-opencv \
      libboost-dev libboost-python-dev pybind11-dev python3-pybind11 \
 && rm -rf /var/lib/apt/lists/*

# Python from apt, as on the host; uv adds only what apt lacks (docker/uv.lock).
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-numpy python3-scipy python3-matplotlib python3-pandas python3-sympy \
      python3-empy=3.3.4-2 python3-jinja2 python3-jsonschema python3-lark python3-lxml \
      python3-yaml python3-serial python3-psutil python3-packaging python3-requests \
      python3-six python3-argcomplete python3-coverage python3-pygments python3-nacl \
      python3-protobuf \
 && rm -rf /var/lib/apt/lists/*

# Everything the host's OpenCV 4.14 configure detected (ffmpeg, hdf5, vtk/viz, ceres/sfm, freetype, java, ...).
RUN apt-get update && apt-get install -y --no-install-recommends \
      libtbb-dev libgtk-3-dev \
      libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
      libjpeg-dev libpng-dev libtiff-dev libwebp-dev libopenexr-dev libopenjp2-7-dev \
      libhdf5-dev libvtk9-dev libceres-dev libgoogle-glog-dev libgflags-dev \
      libfreetype-dev libharfbuzz-dev libdc1394-dev libgphoto2-dev \
      default-jdk-headless \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------------------------
# OpenCV 4.14.0 + contrib, the RUNNING.md recipe (x86 defaults = SSE3 baseline), into /usr/local.
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
      2>&1 | tee /src/opencv-configure.log \
 && cmake --build build -j"$(nproc)" \
 && DESTDIR=/out cmake --install build \
 && cp /src/opencv-configure.log /out/usr/local/share/opencv4/configure.log

# ---------------------------------------------------------------------------------------------
# gz.transport13 Python binding: the vendor gz-transport (13.5.0) ships none, so build it from the same tag.
FROM base AS gz-transport-py
ARG GZ_TRANSPORT_TAG=gz-transport13_13.5.0
WORKDIR /src
RUN git clone --branch ${GZ_TRANSPORT_TAG} --depth 1 https://github.com/gazebosim/gz-transport.git
RUN set +u && source /opt/ros/jazzy/setup.bash \
 && cmake -S gz-transport -B build -G Ninja \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/opt/gz-transport-py \
      -D CMAKE_INSTALL_RPATH=/opt/ros/jazzy/opt/gz_transport_vendor/lib \
      -D SKIP_PYBIND11=OFF -D BUILD_TESTING=OFF \
 && cmake --build build -j"$(nproc)" \
 && cmake --install build \
 && find /opt/gz-transport-py -mindepth 1 -maxdepth 1 ! -name lib -exec rm -rf {} + \
 && find /opt/gz-transport-py/lib -mindepth 1 -maxdepth 1 ! -name python -exec rm -rf {} +

# ---------------------------------------------------------------------------------------------
FROM base AS dev
ARG USERNAME=soni
ARG USER_UID=1000
ARG USER_GID=1000
ARG USER_HOME=/home/soni

# ros-dev-tools and the colcon extensions the host has; --allow-overriding comes from override-check.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-dev-tools \
      python3-colcon-override-check \
      python3-colcon-installed-package-information \
      python3-colcon-mixin \
      python3-pytest python3-pytest-cov python3-pytest-timeout \
 && rm -rf /var/lib/apt/lists/*

COPY --from=opencv /out/usr/local/ /usr/local/
COPY --from=gz-transport-py /opt/gz-transport-py /opt/gz-transport-py
RUN ldconfig

# uv venv over the system Python: apt modules stay visible, uv.lock adds the rest.
COPY --from=uv /uv /uvx /usr/local/bin/
COPY pyproject.toml uv.lock /opt/fxteso-env/
ENV UV_PROJECT_ENVIRONMENT=/opt/venv UV_PYTHON=/usr/bin/python3 UV_PYTHON_DOWNLOADS=never UV_LINK_MODE=copy
RUN uv venv --system-site-packages /opt/venv \
 && cd /opt/fxteso-env && uv sync --frozen --no-install-project --no-cache \
 && printf '%s\n' /opt/ros/jazzy/opt/gz_msgs_vendor/lib/python /opt/gz-transport-py/lib/python \
      > /opt/venv/lib/python3.12/site-packages/gz_vendor.pth
ENV PATH=/opt/venv/bin:${PATH} VIRTUAL_ENV=/opt/venv PYTHONNOUSERSITE=1

# Same user as the host; the base image's uid-1000 'ubuntu' user is removed first.
RUN userdel -r ubuntu 2>/dev/null || true \
 && groupadd -g ${USER_GID} ${USERNAME} \
 && useradd -m -d ${USER_HOME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash \
      -G dialout,plugdev,video,sudo ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && mkdir -p ${USER_HOME}/.ccache && chown -R ${USER_UID}:${USER_GID} ${USER_HOME}

COPY entrypoint.sh /usr/local/bin/fxteso-entrypoint
COPY bashrc /etc/fxteso.bashrc
RUN echo 'source /etc/fxteso.bashrc' >> /etc/bash.bashrc
COPY manifest.sh /usr/local/bin/fxteso-manifest

ENV CCACHE_DIR=${USER_HOME}/.ccache
USER ${USERNAME}
ENTRYPOINT ["/usr/local/bin/fxteso-entrypoint"]
CMD ["sleep", "infinity"]
