# Use the official ROS2 Humble base image
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install System Dependencies (FFmpeg, V4L2 utils, Git, and Python math libs)
RUN apt-get update && apt-get install -y \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    v4l-utils \
    git \
    python3-pip \
    python3-transforms3d \
    && rm -rf /var/lib/apt/lists/*

# 2. Install ROS 2 Packages
RUN apt-get update && apt-get install -y \
    ros-humble-usb-cam \
    ros-humble-v4l2-camera \
    ros-humble-rqt-image-view \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# 3. Create Workspace and Clone the Aruco Repo
WORKDIR /ros2_ws/src
# Use the specific branch if needed, but main is usually fine
RUN git clone https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation.git

# 4. Install dependencies and Build
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    # We add -r to continue even if a key is missing, and explicitly ignore tf2_transformations
    rosdep install --from-paths src --ignore-src -y -r --rosdistro humble && \
    colcon build --symlink-install

# 5. Automatically source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
