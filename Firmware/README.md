# Arduino for Microcontroller Interface (ESP32)

## Directory Tree view

This section describes the structure of the Firmware directory, which contains all microcontroller programs used in the robot system

```bash
├── GNSS                        # SparkFun ZED-F9P GNSS module
│   ├── zedf9p_esp32            # microcontroller program for ESP32 micromod
│   │   ├── APK3.log            # log file for accuracy testing
│   │   ├── secrets.h           # wifi credencial file. DO NOT COMMIT 
│   │   └── zedf9p_esp32.ino    # arduino file
│   └── zedf9p_ucenter          # python program for ucentral port serial data
│       ├── gnss_publisher.py   # python program
│       └── requirements.txt    # requirement dependencies for pip 
├── Robots                      # main Microcontroller programs to control robots
│   ├── MiniATV                 # main ESP32 program of the MiniATV
│   └── Snower                  # Main ESP32 program of the Snower
│       ├── RobotDriveControl.c # diff drive source file 
│       ├── RobotDriveControl.h # diff drive header file 
│       ├── Snower.ino          # main arduino program for Snower
│       └── wifi_secrets.h      # wifi credencial file. DO NOT COMMIT 
├── Trigger                     # trigger button for the dataset project
│   ├── Trigger.ino             # arduino program for triger
│   └── wifi_secrets.h          # wifi credencial file. DO NOT COMMIT 
└── Ultrasonic                  # Ultrasonic sensor array module 
    └── Ultrasonic.ino          # arduino program for Ultrasonic module

```

## Prevent Committing Credentials

To ensure sensitive files are not accidentally committed, run the following commands to ignore modifications locally:
```bash
git update-index --assume-unchanged Firmware/GNSS/zedf9p_esp32/secrets.h
git update-index --assume-unchanged Firmware/Robots/Snower/wifi_secrets.h
```

## Install Arduino IDE
You can use _Ubuntu Sofware_ -application for installing Arduino IDE
or follow:

[https://www.arduino.cc/en/guide/linux](https://www.arduino.cc/en/guide/linux)
or:
[https://ubuntu.com/tutorials/install-the-arduino-ide#1-overview](https://ubuntu.com/tutorials/install-the-arduino-ide#1-overview)

## Arduino IDE for ESP32 setup
You can ignore this if you can upload code to ESP32 using Arduino IDE without any issue. 
1. File -> Preferences
2. Edit "Additional Board Manager URLs"
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json
```
3. Tools -> Board -> Boards Manager. Install ESP32 (by Espressif Systems)
4. Tools -> Board -> esp32 -> Select ESP32 Dev Module
5. Install pip
``` bash
sudo apt install python3-pip
```
6. Install pyserial
``` bash
pip install pyserial
```
7. Now try to upload simple code to the ESP32 and test.

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