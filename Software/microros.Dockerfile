FROM microros/micro-ros-agent:humble

# install ros package
RUN apt-get update
RUN apt-get install -y ros-$ROS_DISTRO-rmw-zenoh-cpp
# RUN apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

RUN rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["serial", "--dev", "/dev/ttyUSB0"]
