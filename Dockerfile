# Dockerfile 

# --- 1. IMAGE BASE ---
# Get ROS2 Humble image base.
FROM osrf/ros:humble-desktop

# --- 2. CONFIGURE THE ENVIRONMENT ---
SHELL ["/bin/bash", "-c"]

# --- 3. DEPENDECIES INSTALLATIONS ---
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    wget \
    xterm \
    python3-pip \
    ros-humble-turtlebot3* \
    ros-humble-xacro && \
    rm -rf /var/lib/apt/lists/*

# --- 4. DOWNLOAD EXTERNAL RESORUCES (MODELS AND WORLD)---
WORKDIR /
RUN git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git

# --- 5. ENVIRONMENT VARIABLE CONFIGURATION ---
# We want Gazebo to find models and worlds
ENV GZ_SIM_RESOURCE_PATH=/gazebo_models_worlds_collection
ENV MESA_GL_VERSION_OVERRIDE=4.1

# --- 6. Create and copy the application to our workspace---
WORKDIR /ros2_ws
COPY ./ros2_ws/src ./src

# --- 7. BUILD THE ROS2 WORKSPACE ---
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# --- 8. CONFIGURE THE ENTRY POINT ---
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# --- 9. SET COMMAND PROMPT ---
CMD ["ros2", "launch", "bump_and_go_pkg", "start_demo.launch.py"]
