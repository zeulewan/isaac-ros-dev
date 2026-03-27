FROM nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64

# Remove broken yarn repo from base image, add standard ROS 2 apt repo
RUN rm -f /etc/apt/sources.list.d/yarn.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list

# Same packages as Carter, plus CycloneDDS + unitree_ros2 deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-isaac-ros-apriltag \
    ros-jazzy-isaac-ros-visual-slam \
    ros-jazzy-isaac-ros-nvblox \
    ros-jazzy-foxglove-compressed-video-transport \
    ros-jazzy-ffmpeg-encoder-decoder \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosidl-generator-dds-idl \
    ffmpeg \
    libyaml-cpp-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create admin user
RUN useradd -m -s /bin/bash -G sudo admin \
    && echo "admin ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Clone and build unitree_ros2 message definitions (G1 uses unitree_hg)
RUN cd /opt && git clone https://github.com/unitreerobotics/unitree_ros2.git \
    && cd unitree_ros2/cyclonedds_ws \
    && /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# CycloneDDS as default RMW (replaces FastDDS to match Unitree's DDS)
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
    > /etc/profile.d/cyclonedds.sh \
    && echo 'source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash' \
    >> /etc/profile.d/cyclonedds.sh \
    && chmod +x /etc/profile.d/cyclonedds.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Auto-start entrypoint scripts
COPY entrypoint/ /usr/local/bin/scripts/entrypoint_additions/
RUN chmod +x /usr/local/bin/scripts/entrypoint_additions/*.sh

WORKDIR /workspaces/isaac_ros-dev
