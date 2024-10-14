# Arduino sketch for ATV Interface (ESP32)


## Install Arduino IDE
You can use _Ubuntu Sofware_ -application for installing Arduino IDE
or follow:

[https://www.arduino.cc/en/guide/linux](https://www.arduino.cc/en/guide/linux)
or:
[https://ubuntu.com/tutorials/install-the-arduino-ide#1-overview](https://ubuntu.com/tutorials/install-the-arduino-ide#1-overview)


## Arduino IDE configuration 
You have to make following changes in Arduino IDE to make code work:
1. **Boards Manager -> esp32 by Expressif Systems (select 2.0.2 version)**. Otherwise you get an error in ledcSetup() and ledcAttachPin()
2. in library manager, Search **ESP32TimerInterrupt (select 2.3.0 version)**
3. Download zip file from this git repository, branch foxy [https://github.com/micro-ROS/micro_ros_arduino/tree/foxy](https://github.com/micro-ROS/micro_ros_arduino/tree/foxy)
4. **Sketsch -> add Zip Library -> Select the Downloaded zip file**


## Jetson Nano UART:
Before You Start
(Copied from some where nVidia Developer Forum) The stock Jetson Nano starts a console on the ttyTHS1 serial port at startup through a service. The script that starts the service is nvgetty.sh which launches getty. The script is located in /etc/systemd. While this does not conflict with the script presented here, consider disabling the console if you are using the serial port to avoid conflicts. Note that normal udev rules will be overridden by the console while the service is running. To disable the console:
```
$ systemctl stop nvgetty
$ systemctl disable nvgetty
$ udevadm trigger
# You may want to reboot instead
```
> **Warning:**
> This functionality has not been tested on the Jetson Nano Orin. However, it has been reported as not working on the Jetson Nano in the Lapland Robotics Projects.