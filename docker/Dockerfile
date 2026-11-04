# The development environment: ROS 2 Jazzy on Ubuntu 24.04, the platform PX4 supports. The image
# already has colcon and rosdep, and its entrypoint sources ROS 2's setup for every command.
FROM ros:jazzy-ros-base-noble

# Every package's dependencies, as its package.xml declares them. The build context holds only the
# package.xml files (tools/dev.sh), so a change to the code doesn't rebuild this layer.
COPY src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /tmp/src --ignore-src --rosdistro jazzy -y \
    && rm -rf /var/lib/apt/lists/* /tmp/src

# Gazebo's transport finds its peers by multicast on this address. Everything runs in this one
# container, and multicast on a CI runner's container network reaches nobody.
ENV GZ_IP=127.0.0.1

# Models Gazebo's example worlds fetch from Fuel as they load, fetched once here, so the tests run
# offline and on the same model every time. Loading the world fetches them where it will look for
# them: `gz fuel download` caches under another name.
ENV GZ_FUEL_CACHE_PATH=/opt/gz-fuel
RUN mkdir -p "$GZ_FUEL_CACHE_PATH" \
    && . /opt/ros/jazzy/setup.sh \
    && gz sim -s -r --iterations 1 auv_controls.sdf
