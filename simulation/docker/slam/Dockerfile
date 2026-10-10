# slam_toolbox (online sync) + rosbridge for the MuJoCo warehouse simulator.
FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-slam-toolbox ros-jazzy-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Keep DDS discovery inside the container; the simulator talks over rosbridge (websocket).
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

COPY launch /qcw/launch
COPY config /qcw/config

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch /qcw/launch/qcw_slam.launch.py"]
