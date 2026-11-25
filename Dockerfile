ARG BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-pip python3-opencv python3-numpy python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp ros-humble-sensor-msgs-py \
    libcyclonedds-dev && rm -rf /var/lib/apt/lists/*
# Official, public Unitree SDK2 Python. Kept outside this repository.
RUN python3 -m pip install --no-cache-dir cyclonedds==0.10.2 && \
    python3 -m pip install --no-cache-dir --no-deps git+https://github.com/unitreerobotics/unitree_sdk2_python.git
WORKDIR /workspace
COPY src/ /workspace/src/
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch go2_follower_bringup follower.launch.py network_interface:=${UNITREE_NET_IFACE:-eth0}"]
