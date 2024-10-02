# Use the official ROS2 Foxy base image
FROM osrf/ros:jazzy-desktop

ENV WORKSPACE=/workspace

# Create a workspace directory
RUN mkdir -p ${WORKSPACE}/src

# Change to workspace
WORKDIR ${WORKSPACE}

# Copy the package into the workspace
COPY . ${WORKSPACE}/src/simplesim

# Rosdep install
RUN /bin/bash -c "apt-get update -qq && rosdep install --from-paths src --ignore-src -r -y"

# Build the package
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Set the entrypoint to run ROS2 when the container starts
ENTRYPOINT ["/bin/bash", "-c", "source /workspace/install/setup.bash && ros2 run simplesim simplesim_node"]

