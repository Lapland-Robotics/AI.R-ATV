FROM ros:humble

# Install dependencies
RUN apt-get update
RUN apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
RUN apt-get install -y libsdl2-dev
RUN apt-get install -y udev
RUN apt-get install -y python3-pip
RUN pip install pcl
RUN pip install seekcamera-python
RUN pip install Pillow
    
RUN rm -rf /var/lib/apt/lists/*
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

# ROS setup
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///app/cyclonedds-config.xml

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY cyclonedds-config.xml /app/
COPY ros2_ws ros2_ws/

RUN cd ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

ADD ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

# Create directory for SDK
RUN mkdir -p /opt/seekthermal-sdk

# Copy the SDK from local machine
COPY drivers/src/Seek_Thermal_SDK_4.4.2.20 /opt/seekthermal-sdk/

# Determine architecture and set variables
ARG ARCH=aarch64-linux-gnu  # Change this based on architecture
ENV SDK_ARCH=${ARCH}

# Install SDK manually (Linux manual installation steps)
WORKDIR /opt/seekthermal-sdk/${SDK_ARCH}
RUN cp lib/libseekcamera.so /usr/local/lib
RUN cp lib/libseekcamera.so.4.4 /usr/local/lib
RUN cp -r include/* /usr/local/include
RUN cp driver/udev/10-seekthermal.rules /etc/udev/rules.d
RUN chmod +x bin/*

# Update library cache
RUN ldconfig

# Create udev rules directory if it doesn't exist
RUN mkdir -p /etc/udev/rules.d

# Set up environment variables
ARG LD_LIBRARY_PATH=/usr/local/lib
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
ENV PATH=/opt/seekthermal-sdk/${SDK_ARCH}/bin:${PATH}

# Create a non-root user for ROS (good practice)
# RUN useradd -m -u 1000 -G dialout,video,plugdev rosuser && \
#     chown -R rosuser:rosuser /workspace

# USER rosuser

# Source ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Test command to verify SDK installation
CMD ["ros2", "launch", "data_collector", "data_collector.launch.py"]