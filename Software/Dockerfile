FROM ros:humble

# install ros package
RUN apt-get update
RUN apt-get install -y python3-opencv
RUN apt-get install -y ros-$ROS_DISTRO-cv-bridge
RUN apt-get install -y ros-$ROS_DISTRO-tf-transformations
RUN apt-get install -y ros-$ROS_DISTRO-xacro
RUN apt-get install -y ros-$ROS_DISTRO-robot-localization
RUN apt-get install -y ros-$ROS_DISTRO-navigation2
RUN apt-get install -y ros-$ROS_DISTRO-nav2-bringup
RUN apt-get install -y ros-$ROS_DISTRO-slam-toolbox
RUN apt-get install -y ros-$ROS_DISTRO-pointcloud-to-laserscan
RUN apt-get install -y ros-$ROS_DISTRO-rmw-zenoh-cpp
# RUN apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

RUN rm -rf /var/lib/apt/lists/*
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_ROUTER_CONFIG_URI=file:///app/zenoh-router-config.json5
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY zenoh-router-config.json5 /app/
COPY ros2_ws ros2_ws/

RUN cd ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

ADD ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "robot", "run.launch.py"]
