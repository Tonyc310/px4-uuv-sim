# The development environment: ROS 2 Jazzy on Ubuntu 24.04, the platform PX4 supports. The image
# already has colcon and rosdep, and its entrypoint sources ROS 2's setup for every command.
FROM ros:jazzy-ros-base-noble

# Gazebo Harmonic through ROS 2's packages, and what PX4's simulation build needs besides: the list
# in PX4's Tools/setup/ubuntu.sh, less the Gazebo it would install from OSRF's repository and the
# GStreamer plugins that only stream camera video. PX4 builds against this Gazebo, so it comes
# before PX4; package.xml declares it too.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        bc cppzmq-dev libeigen3-dev libgstreamer-plugins-base1.0-dev libopencv-dev libunwind-dev \
        libxml2-utils ninja-build pkg-config protobuf-compiler python3-dev python3-venv rsync zip \
    && rm -rf /var/lib/apt/lists/*

# PX4 at a release tag. Its Python build tools get an environment of their own: their requirements
# would replace Ubuntu's NumPy, which ROS 2 uses.
ARG PX4_VERSION=v1.17.0
RUN git clone --branch "$PX4_VERSION" --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
    && python3 -m venv --system-site-packages /opt/px4-venv \
    && /opt/px4-venv/bin/pip install --no-cache-dir \
        -r /opt/PX4-Autopilot/Tools/setup/requirements.txt

# PX4's software-in-the-loop build, with its Gazebo bridge and plugins.
ENV PX4_BUILD=/opt/PX4-Autopilot/build/px4_sitl_default
RUN . /opt/ros/jazzy/setup.sh \
    && PATH="/opt/px4-venv/bin:$PATH" make -C /opt/PX4-Autopilot px4_sitl

# The bridge from PX4 to ROS 2: the Micro XRCE-DDS Agent, at the version PX4's ROS 2 guide uses.
# PX4's client sends it uORB topics over UDP, and it publishes them on DDS.
ARG XRCE_AGENT_VERSION=v2.4.3
RUN git clone --branch "$XRCE_AGENT_VERSION" --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cmake -S /tmp/agent -B /tmp/agent/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/agent/build -j"$(nproc)" \
    && cmake --install /tmp/agent/build \
    && ldconfig \
    && rm -rf /tmp/agent

# PX4's messages as ROS 2 types, which must match PX4's release, built into a workspace the
# repository's packages build on: px4_msgs's release/1.17 branch, at a fixed commit.
ARG PX4_MSGS_COMMIT=86d8239e962f6939e05c3737784f60c02fa884db
RUN mkdir -p /opt/px4_ws/src/px4_msgs \
    && cd /opt/px4_ws/src/px4_msgs \
    && git init --quiet \
    && git fetch --quiet --depth 1 https://github.com/PX4/px4_msgs.git "$PX4_MSGS_COMMIT" \
    && git checkout --quiet FETCH_HEAD \
    && . /opt/ros/jazzy/setup.sh \
    && cd /opt/px4_ws \
    && colcon build --merge-install

# Every package's dependencies, as its package.xml declares them; px4_msgs is the workspace above.
# The build context holds only the package.xml files (tools/dev.sh), so a change to the code
# doesn't rebuild this layer.
COPY src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /tmp/src /opt/px4_ws/src --ignore-src --rosdistro jazzy -y \
    && rm -rf /var/lib/apt/lists/* /tmp/src

# Gazebo's transport only talks within a partition, named by default after the host and the user.
# The container runs as the calling user, who may have no name in it (CI's runner doesn't), and
# then nothing connects.
ENV GZ_PARTITION=px4-uuv-sim

# Models Gazebo's example worlds fetch from Fuel as they load, fetched once here, so the tests run
# offline and on the same model every time. Loading the world fetches them where it will look for
# them: `gz fuel download` caches under another name.
ENV GZ_FUEL_CACHE_PATH=/opt/gz-fuel
RUN mkdir -p "$GZ_FUEL_CACHE_PATH" \
    && . /opt/ros/jazzy/setup.sh \
    && gz sim -s -r --iterations 1 auv_controls.sdf

# Every command runs with ROS 2 and the px4_msgs workspace set up.
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
