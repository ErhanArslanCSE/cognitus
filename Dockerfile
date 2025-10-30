# COGNITUS Development Environment
# Matches LIMO Pro: Ubuntu 20.04 + ROS2 Foxy

FROM osrf/ros:foxy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install essential packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # ROS2 packages
    ros-foxy-vision-msgs \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-xacro \
    # Tools
    git \
    wget \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || true

# Workspace
WORKDIR /workspace

# Copy source
COPY src/ /workspace/src/

# Build workspace
RUN . /opt/ros/foxy/setup.sh && \
    rosdep update || true && \
    rosdep install --from-paths src --ignore-src -y || true && \
    colcon build --symlink-install

# Environment setup
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

CMD ["bash"]
