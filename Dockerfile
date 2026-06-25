# kompass-sim — ROS 2 Jazzy + Webots, ready to run

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Build tooling (most already present in ros-base).
RUN apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Resolve ROS dependencies
COPY package.xml /ros2_ws/src/kompass_sim/package.xml
RUN apt-get update \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update --rosdistro "$ROS_DISTRO" \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO" \
    && rm -rf /var/lib/apt/lists/*

# install webots dependency libxcb-cursor0
RUN apt-get update && apt-get install -y --no-install-recommends \
        libxcb-cursor0 \
        wget \
    && rm -rf /var/lib/apt/lists/*

# Pull Webots into the image
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && WEBOTS_VER="$(python3 -c 'from webots_ros2_driver.utils import WebotsVersion; print(WebotsVersion.minimum().short())')" \
    && echo "Pre-fetching Webots ${WEBOTS_VER} archive (resumable)..." \
    && mkdir -p /root/.ros \
    && wget --continue --tries=100 --retry-connrefused --waitretry=5 \
            --timeout=30 --read-timeout=30 --progress=dot:giga \
            -O "/root/.ros/webots-${WEBOTS_VER}-x86-64.tar.bz2" \
            "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VER}/webots-${WEBOTS_VER}-x86-64.tar.bz2" \
    && echo "y" | python3 -c \
        "from webots_ros2_driver.utils import handle_webots_installation; handle_webots_installation()" \
    && ln -s /root/.ros/webots*/webots /usr/local/webots

ENV WEBOTS_HOME=/usr/local/webots

# Verify that the baked-in Webots is in place and that webots_ros2 will detect it at runtime instead of re-downloading
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && cat /usr/local/webots/resources/version.txt \
    && python3 -c "from webots_ros2_driver.utils import get_webots_home; h = get_webots_home(); print('Webots detected at:', h); assert h, 'Webots NOT detected by webots_ros2'"

# Python dependency for webots_ros2's bundled URDF importer
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pil \
    && rm -rf /var/lib/apt/lists/*

# Build the package.
COPY . /ros2_ws/src/kompass_sim
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && colcon build --merge-install

# Entrypoint sources ROS 2 + the workspace before running the command.
RUN printf '%s\n' \
        '#!/usr/bin/env bash' \
        'set -e' \
        'source /opt/ros/jazzy/setup.bash' \
        'source /ros2_ws/install/setup.bash' \
        'exec "$@"' \
        > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

# Webots needs USER to be set
ENV USER=root

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "kompass_sim", "webots_turtlebot3.launch.py"]
