FROM microros/micro-ros-agent:foxy

# install ros package
RUN apt-get update
RUN apt-get install -y ros-${ROS_DISTRO}-demo-nodes-cpp 
RUN apt-get install -y ros-${ROS_DISTRO}-demo-nodes-py
RUN apt-get install -y ros-${ROS_DISTRO}-rqt
RUN rm -rf /var/lib/apt/lists/*

RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

ENV TOPIC="/ouster/imu"

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY ros2_ws ros2_ws/
RUN cd ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

COPY ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "run", "esp32_microros_controller", "esp32_controller_node"]

# launch ros package
# CMD ["sleep", "infinity"]
