FROM osrf/ros:humble-desktop

# 1. Install System Dependencies (Git, X11, Foxglove)
RUN apt-get update && apt-get install -y \
    git \
    x11-apps \
    x11-xserver-utils \
    ros-humble-foxglove-bridge \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. Ensure ROS is sourced for every new bash session
# This makes it so you don't have to 'source' manually in every terminal
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set up environment variables so the build works without manual sourcing
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/humble

# 3. Setup Workspace and Clone Repo
WORKDIR /ros2_ws/src

# We use a trick here: if the folder exists, we remove it to ensure a clean clone
RUN git clone https://github.com/aojedao/ATMOSTestbench.git .

# 4. CRITICAL CLEANUP: Remove the pre-built folders from the repo
RUN find . -type d \( -name "build" -o -name "install" -o -name "log" \) -exec rm -rf {} +

# 5. Install ROS Dependencies
WORKDIR /ros2_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 6. Build the workspace
# We use 'bash -c' to ensure the ROS environment is loaded for the build
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add the workspace to your bashrc as well
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the default working directory
WORKDIR /ros2_ws