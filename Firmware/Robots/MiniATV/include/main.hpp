#ifndef MAIN_HPP
#define MAIN_HPP

extern "C"{
  #include "ATV.h"
}
#include "pwmread_rcfailsafe.hpp"
#include <Arduino.h>
#include <stdio.h>
#include <ESP32TimerInterrupt.h>    // 2.3.0 version
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

/* ESP32 pin definition */
#define SteeringDirPin 12      // Steering Motor Direction
#define SteeringPulsePin 13    // Steering Motor drive pulse
#define SteeringEnablePin 9    // Steering Motor Enable, Outside leads to ground and +5V
#define SteeringPotPin 36      // Potentiometer wiper (middle terminal) connected to analog pin 0
#define SteeringLimitSWPin 33  // Hall-sensor input for Limit steering angle
#define DrivingDirPin 25       // Direction output for Driver motor controlle
#define DrivingSpeedPWMPin 26  // Speed output for Driver motor controller
#define DrivingEnablePin 27    // Driving Motor Controller enable (NOT IN USE)
#define FRWheelPulsePin 15     // Front Right Wheel rotation pulse, Use Hall Sensor with 8 magnets
#define FLWheelPulsePin 4      // Front Left Wheel rotation pulse, Use Hall Sensor with 8 magnets
#define HWIsolatorEnablePin 2  // Constant for Hardware control. Enable or disable Isolator IC's on PCB
#define SafetySWPin 32         // Safety HW If '0' = safe
#define ModeSwitchPin 5        // switch between RC mode <-> autonomous mode

/* Constants for Steering  */
#define Steering_Deadband 2       // Acceptable steering error (here named "deadband"), to avoid steering jerking (bad steering position measurement and poor stepper motor drive)
#define Steering_Middlepoint 50   // Steering Command Middle point
#define Steering_Speed 10000      // Change Steering Speed Fast (half pulse 500 => 2*500 = 1000) 1000us ~ 1000Hz
#define Max_Half_Step_Count 30
#define ADC_Bits 4095
#define Left 0
#define Right 1

/* Constants for Driving  */
#define Driving_Speed_Middlepoint 50  // Dummy engineering constant for setting middlepoint of Speed Command
#define Driving_Speed_Duty_Coef 20    // Dummy engineer Coefficient for scale PWM duty cycle
#define Forward 0                     // Forward = 0
#define Backward 1                    // Backward (Reverse) = 1

/* Constants for ROS Steering command calculation */
#define ROS_Steering_Command_Slope -111.0
#define ROS_Steering_Command_yIntercept 50.0
#define ROS_Speed_Command_Slope 64  //53.0 //3.3
#define ROS_Speed_Command_yIntercept 50.0

/* Constants for Speed measurement and Odometry calculation  */
#define Wheel_Circumference 900.0         // Wheels circumference in milli meters [mm]
#define Wheel_Pulse_Magnet_Count 16       // Magnet count in one wheel for measuring wheel pulses
#define Speed_Calculation_Interval 200.0  // Speed Calculation Interval in milli second [ms]
//long Speed_Measured = 50;               // Measured Driving Speed 0-100, 0 = Full Reverse, 0 = Stop and 100 = Full Forward

//#define ROS_Interval 1000  // FOR DEBUGING VIA SERIAL PORT
#define ROS_Interval 200            // ROS Commands update interval in milli second [ms]
#define ROS_Max_Missing_Packets 10  // How many (ROS_Interval) subsequent ROS command not receive and then reject ROS control, If ROS_Interval = 100 ms and ROS_Max_Missing_Packets = 10 than Max silent time is 100 ms * 10 = 1s

/*ROS2 Constants*/
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

/* Dummy engineering constants for setting Slope and y-intercept for calculation: Real Speed(ROS_Speed_Measured) * Slope + y-intercept */
#define Speed_Measurement_Slope 12.0        // Example 3m/s*12+50 = 86
#define Speed_Measurement_yIntercept 50.0

/* Constants for RC "pwm reading"  */
#define RC_Minimum 1000  // RC minimum is 1000us
#define RC_Scaler 10     // Dummy engineer divider for scale RC signal to 0(neg max) - 50(middlepoint) - 100(pos max) => (RC_input - RC_min)/RC_Scaler => (1500-1000)/10 = 50

/* ROS SIMULATOR WITHOUT REAL INTERFACE TO ATV */
//#define Simulation    // Compiler directives for simulation or not

void error_loop();
void error_debug(char error_cause[256]);
void Driving();
void Safety_Switch();
void IRAM_ATTR Steering_Limit();
void IRAM_ATTR Front_Right_Wheel_Pulse();
void IRAM_ATTR Front_Left_Wheel_Pulse();
bool IRAM_ATTR Steering_Pulse_Interrupt(void* param);
bool IRAM_ATTR Speed_Calculation_Interrupt(void* param);
bool IRAM_ATTR Steering_Calculation_Interrupt(void* param);
void ctrlCmdCallback(const void *msgin);
void generate_debug_data();


#endif // MAIN_HPP