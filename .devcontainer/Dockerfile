FROM osrf/ros:jazzy-desktop
ARG PX4_TAG=v1.17.0
ARG XRCE_AGENT_TAG=v2.4.3
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo git python3-pip python3-vcstool python3-colcon-common-extensions \
      ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Micro XRCE-DDS Agent (bridges PX4 to ROS 2)
RUN git clone -b ${XRCE_AGENT_TAG} --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cmake -S /tmp/agent -B /tmp/agent/build \
    && cmake --build /tmp/agent/build -j"$(nproc)" \
    && cmake --install /tmp/agent/build && ldconfig /usr/local/lib && rm -rf /tmp/agent

# PX4 source at the pinned tag (own step so it is cached)
RUN git clone --recursive -b ${PX4_TAG} https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot

# Download tools PX4's setup script expects (wget for the Gazebo package key)
RUN apt-get update && apt-get install -y --no-install-recommends \
      wget curl gnupg lsb-release ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# PX4 toolchain and Gazebo, simulation only (no flight-controller cross-compiler).
# Keep Ubuntu's numpy: ROS 2 depends on it and pip cannot replace it.
RUN printf "[global]\nconstraint = /etc/pip-constraints.txt\n" > /etc/pip.conf \
    && echo "numpy==1.26.4" > /etc/pip-constraints.txt \
    && bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
    && rm /etc/pip.conf /etc/pip-constraints.txt \
    && chown -R ubuntu:ubuntu /opt/PX4-Autopilot

RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc
