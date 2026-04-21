FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 1. Consolidated System Dependencies
RUN apt-get update && apt-get install -y \
    ffmpeg libavcodec-dev libavformat-dev libswscale-dev v4l-utils git \
    python3-pip python3-transforms3d \
    # Added from your installed list
    build-essential cmake autoconf automake \
    curl vim wget zip unzip \
    default-jdk default-libmysqlclient-dev unixodbc-dev \
    gstreamer1.0-libav gstreamer1.0-plugins-bad gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*

# 2. Install ROS 2 Packages
RUN apt-get update && apt-get install -y \
    ros-humble-usb-cam ros-humble-v4l2-camera \
    ros-humble-rqt-image-view ros-humble-cv-bridge \
    ros-humble-image-transport ros-humble-image-transport-plugins \
    ros-humble-tf2-geometry-msgs ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# 3. Workspace Setup
WORKDIR /ros2_ws/src

# Clone the repo. 
# We use the '.' to ensure the contents go into /ros2_ws/src directly
RUN git clone https://github.com/aojedao/ATMOSTestbench.git .

# 3. Install dependencies for the cloned repo
# Move back to the workspace root so rosdep can see the 'src' folder
WORKDIR /ros2_ws

#RUN apt-get update && \
#    rosdep update && \
#    rosdep install --from-paths src --ignore-src -r -y && \
#    rm -rf /var/lib/apt/lists/*

# 4. Build the workspace
#RUN source /opt/ros/humble/setup.bash && \
#    colcon build --symlink-install