# Arduino sketch for ATV Interface (ESP32)


## Install Arduino IDE
You can use _Ubuntu Sofware_ -application for installing Arduino IDE
or follow:

[https://www.arduino.cc/en/guide/linux](https://www.arduino.cc/en/guide/linux)

or:

[https://ubuntu.com/tutorials/install-the-arduino-ide#1-overview](https://ubuntu.com/tutorials/install-the-arduino-ide#1-overview)


## Arduino IDE configuration 
You have to make following changes in Arduino IDE to make code work:
1. Board: BoardManager: "esp32" version 2.0.13 then use Board "ESP32 Dev Module" Otherwise you get an error in ledcSetup() and ledcAttachPin()
2. in library manager, use ESP32TimerInterrupt version 1.5.0
3. in library manager, use ROSserial version 0.9.1
4. Go here https://github.com/enwaytech/ros_lib_arduino/tree/master,download ZIP, and then move ackermann_msgs folder from src folder
to "C:\ArduinoIDE\libraries\Rosserial_Arduino_Library\src" or wherever your Arduino IDE library folder is located


## Make ros_lib into the Arduino Environment without download
[http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

The preceding installation steps created the necessary libraries, now the following will create the ros_lib folder that the Arduino build environment needs to enable Arduino programs to interact with ROS.
In the steps below, <sketchbook> is the directory where the Linux Arduino environment saves your sketches. Typically this is a directory called sketchbook or Arduino in your home directory. e.g cd ~/Arduino/libraries
Ros_lib installation instructions are different for groovy source (catkin) than for earlier (rosbuild) or binary releases. Be sure you've selected the correct build system - catkin for a groovy source build, rosbuild otherwise.
Note: you have to delete libraries/ros_lib, if present, in order to regenerate as its existence causes an error. "rosrun rosserial_arduino make_libraries.py" creates the ros_lib directory.
```
$ cd <sketchbook>/libraries
$ rm -rf ros_lib
$ rosrun rosserial_arduino make_libraries.py ros_lib
```
*Note: Currently you can install the Arduino libaries directly in the Arduino IDE. Just open the Library Manager from the IDE menu in Sketch -> Include Library -> Manage Library. Then search for "rosserial". This is useful if you need to work on an Arduino sketch but don't want to setup a full ROS workstation.


## General Notes when using ESP32 and rosserial in ESP32


## ESP32 HW Timer
```
#include "ESP32TimerInterrupt.h"
``` 

## ESP32 rebooting when Arduino IDE ROS libraries used:
Serial port monitor show:
  
```
Rebooting...
ets Jun  8 2016 00:22:57

rst:0xc (SW_CPU_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0018,len:4
load:0x3fff001c,len:1216
ho 0 tail 12 room 4
load:0x40078000,len:10944
load:0x40080400,len:6388
entry 0x400806b4
assertion "Invalid mbox" failed: file "/home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/lwip/lwip/src/api/tcpip.c", line 374, function: tcpip_send_msg_wait_sem
abort() was called at PC 0x400d49c7 on core 1

ELF file SHA256: 0000000000000000

Backtrace: 0x40084cb4:0x3ffb1de0 0x40084f31:0x3ffb1e00 0x400d49c7:0x3ffb1e20 0x400e44b3:0x3ffb1e50 0x400e3f4d:0x3ffb1e80 0x400e4100:0x3ffb1ea0 0x400e7654:0x3ffb1ee0 0x400d2632:0x3ffb1f00 0x400d2032:0x3ffb1f50 0x400d1b6b:0x3ffb1f80 0x400d2efa:0x3ffb1fb0 0x40085f32:0x3ffb1fd0

```
  
HOW TO AVOID THIS:
Thank you LorenzF in [https://github.com/espressif/arduino-esp32/issues/4807](https://github.com/espressif/arduino-esp32/issues/4807)

On Arduino Ros library ros_lib there is ros.h .

Replace or comment this:
```
#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif
```
with this:
```
#if defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif
```


## CHANGE ROSSERIAL BAUD RATE
You have to change parameters in ArduinoHardware.h in ros_lib.


## Jetson Nano UART: (THIS WONT WORK, DON'T WASTE YOUR TIME)
Before You Start
(Copied from some where nVidia Developer Forum) The stock Jetson Nano starts a console on the ttyTHS1 serial port at startup through a service. The script that starts the service is nvgetty.sh which launches getty. The script is located in /etc/systemd. While this does not conflict with the script presented here, consider disabling the console if you are using the serial port to avoid conflicts. Note that normal udev rules will be overridden by the console while the service is running. To disable the console:
```
$ systemctl stop nvgetty
$ systemctl disable nvgetty
$ udevadm trigger
# You may want to reboot instead
```
