#ifndef MAIN_HPP
#define MAIN_HPP

#include <stdio.h>
#include <ESP32TimerInterrupt.h>    // 2.3.0 version
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include "wifi_secrets.h"
extern "C"{
  #include "RobotDriveControl.h"
}
#include <Arduino.h>
#include "driver/pcnt.h"

/* ESP32 pin definition */
#define CH1RCPin 18 //Remote Control ch1
#define CH2RCPin 19 //Remote Control ch2
#define MotorLeftDirPin  22 //Motor1 Direction
#define MotorRightDirPin  23 //Motor2 Direction
#define MotorLeftPWMPin 25 //Motor1 Speed PWM
#define MotorRightPWMPin 26 //Motor2 Speed PWM
#define MCEnablePin 21 //Motor Controller Enable Pin
#define SpeedSensorLeftPin 32 //Left speed sensor
#define SpeedSensorRightPin 33 //Right speed sensor
 
//Constants
#define FREQ  10000  //AnalogWrite frequency
#define MAX_T 2500 //Max signal threshold
#define MIN_T 500  //Min signal threshold
#define RESOLUTION 8 //PWM resolution (8-bit, range from 0-255)
#define LINEAR_X_DEFAULT 0   // Middle point of speed
#define ANGULAR_Z_DEFAULT 0  // middlepoint of angle
#define GENERAL_BLOCK_FREQUENCY 40   // Odometry publish rate in Hz
#define DEBUG_PUBLISHER_FREQUENCY 2  // Odometry publish rate in Hz
#define SPEED_PUBLISHER_FREQUENCY 40  // Odometry publish rate in Hz
#define TIMER0_COUNT_FREQUENCY 1000000 // Timer0 count frequency
#define PWM_SLOPE 340.00 // Slope of the linearization function (m)
#define PWM_INTERCEPT -17.00 // Intercept of the linearization function (C)
#define MIN_PWM 50 // Minimum PWM value to start the motors
#define MAX_PWM 255 // Maximum PWM value to start the motors
#define RC_PWM_UPPER_THRESHOLD 1600  // Upper threshold for RC PWM
#define RC_PWM_LOWER_THRESHOLD 1400  // Lower threshold for RC PWM
  
/*ROS2 Constants*/
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
 
// Pulse counter units
#define PCNT_LEFT_UNIT PCNT_UNIT_0
#define PCNT_RIGHT_UNIT PCNT_UNIT_1
 
void errorLoop();
bool safePublish(rcl_publisher_t* publisher, void* msg, const char* publisher_name);
boolean isRCActive();
void debugDataPublisher(char final_string[128]);
void cmdVelCallback(const void *msgin);
void IRAM_ATTR CH1_interrupt();
void IRAM_ATTR CH2_interrupt();
void publishSpeed(rcl_timer_t * timer, int64_t last_call_time);
void setupPCNT(pcnt_unit_t unit, int pin);
void microrosInit();
void microrosCleanup();
double mapFloat(int x, double in_min, double in_max, double out_min, double out_max);
void getRC();
void activateMotorController();
int getPWMbySpeed(double speed);
void driving();
#endif