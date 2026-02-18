# This is an auto generated Dockerfile for ros:ros-base
FROM ros:jazzy-ros-core-noble

# Use bash for all RUN commands
SHELL ["/bin/bash", "-c"]

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# Setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base=0.11.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostics \
    ros-jazzy-xacro
    
# Create workspace directory
RUN mkdir -p /home/ubuntu/ros2_ws/src

# Set working directory
WORKDIR /home/ubuntu/ros2_ws

# Automatically source ROS 2 and workspace
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f install/setup.bash ]; then source install/setup.bash; fi" >> /root/.bashrc

# Default command
CMD ["bash"]

