/*
  GNSS module obtain RTCM data from a NTRIP Caster over WiFi and push it over I2C to a ZED-F9x.
  The module is acting as a 'client' to a 'caster'.
*/
#ifndef MAIN_HPP
#define MAIN_HPP

  #include <Arduino.h>
  //The ESP32 core has a built in base64 library but not every platform does
  //We'll use an external lib if necessary.
  #if defined(ARDUINO_ARCH_ESP32)
  #include "base64.h"  //Built-in ESP32 library
  #else
  #include <Base64.h>  //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
  #endif

  #include <WiFi.h>
  #include <stdio.h>
  #include <math.h>
  #include "secrets.h"
  #include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS
  #include <micro_ros_platformio.h>
  #include <rosidl_runtime_c/string_functions.h>
  #include <rcl/rcl.h>
  #include <rcl/error_handling.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <sensor_msgs/msg/nav_sat_fix.h>
  #include <std_msgs/msg/string.h>

  #include <WString.h>

  #define EARTH_RADIUS_CM 637100000.0
  #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
  #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

  void errorLoop();
  double degToRad(double deg);
  double haversineDistance(double lat1, double lon1, double lat2, double lon2);
  void microrosInit();
  void nTripInit();
  void updateGnssVar();
  void publishMsg();
  void debug(const char* format, ...);
  void beginClient();

#endif