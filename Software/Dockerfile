FROM ros:humble

# install ros package
RUN apt-get update
RUN apt-get install -y python3-opencv
RUN apt-get install -y ros-humble-cv-bridge
RUN apt-get install -y ros-humble-tf-transformations
RUN apt-get install -y ros-humble-xacro
RUN apt-get install -y ros-humble-robot-localization
RUN rm -rf /var/lib/apt/lists/*

RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY ros2_ws ros2_ws/
RUN cd ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

ADD ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "atv", "run.launch.py"]
