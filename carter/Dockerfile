FROM nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64

# Remove broken yarn repo from base image, add standard ROS 2 apt repo
RUN rm -f /etc/apt/sources.list.d/yarn.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list

# Isaac ROS GPU perception + Foxglove H.264 + ffmpeg + teleop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-isaac-ros-apriltag \
    ros-jazzy-isaac-ros-visual-slam \
    ros-jazzy-isaac-ros-nvblox \
    ros-jazzy-foxglove-compressed-video-transport \
    ros-jazzy-ffmpeg-encoder-decoder \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Create admin user
RUN useradd -m -s /bin/bash -G sudo admin \
    && echo "admin ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# FastDDS no-SHM XML (fixes host<->container DDS transport)
COPY config/fastdds_no_shm.xml /etc/fastdds_no_shm.xml
RUN echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_no_shm.xml' \
    > /etc/profile.d/fastdds-fix.sh \
    && chmod +x /etc/profile.d/fastdds-fix.sh

# Auto-start entrypoint scripts
COPY entrypoint/ /usr/local/bin/scripts/entrypoint_additions/
RUN chmod +x /usr/local/bin/scripts/entrypoint_additions/*.sh

WORKDIR /workspaces/isaac_ros-dev
