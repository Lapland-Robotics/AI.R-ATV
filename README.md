# AI.R-ATV
This is a ROS2 platform based program designed to control and Monitor ATVs.

## Tree view

```
|-- Documents: All the Documents and Diagrams
|-- Firmware: All the firmware
|    |-- ATV: Main Microprocessor program for all the ATVs
|    |   |-- MiniATV: Mini ATV main Microprocessor(ESP32) program
|    |   |-- Snower: Snower main Microprocessor program
|    |-- GNSS: GNSS module
|    |   |-- zedf9p_esp32: ZED-F9P ESP32 -> microros Approch
|    |   |-- zedf9p_ucenter: ZED-F9P ucentral Serial -> ROS2 ucentral Approch
|    |-- Ultrasonic: Arduino Program for Ultrasonic sensor array
|-- Software: ROS2 project for Main Brain of the ATV (Jetson)
```

## Firmware directory
Low-level Program for Microcontrollers.

## Software directory
ROS2 Software program for High-level functions like decision making and pathfinding algorithms of the ATV.

## Documents
All the Documents and diagrams. 


![Arctic ai Robotics (2)](https://github.com/user-attachments/assets/b88b88a5-4c25-415a-a896-f6bfdef2293f)
 