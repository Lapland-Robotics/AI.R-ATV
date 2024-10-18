#include <stdio.h>
#include <ESP32TimerInterrupt.h>    // 2.3.0 version
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
extern "C"{
  #include "ATV.h"
}

/* ESP32 pin definition */
#define CH1RCPin 18 //Remote Control ch1
#define CH2RCPin 19 //Remote Control ch2
#define Motor1DirPin  22 //Motor1 Direction
#define Motor2DirPin  23 //Motor2 Direction
#define Motor1SpeedPWMPin 25 //Motor1 Speed PWM
#define Motor2SpeedPWMPin 26 //Motor2 Speed PWM
 
//Constants
#define FREQ  490  //AnalogWrite frequency
#define MAX_T 2500 //Max signal threshold
#define MIN_T 500  //Min signal threshold
#define RESOLUTION 8 //PWM resolution (8-bit, range from 0-255)
#define Steering_Middlepoint 0   // Steering Command Middle point
#define Driving_Speed_Middlepoint 0  // Dummy engineering constant for setting middlepoint of Speed Command
#define ROS_Steering_Command_yIntercept 0
#define ROS_Speed_Command_yIntercept 0
#define ROS_Steering_Command_Slope 255
#define ROS_Speed_Command_Slope 255

/* Time variables */
unsigned long CurrentTime = 0;   // Time now in milli seconds [ms]
unsigned long PreviousTime = 0;  // Last iteration time in milli seconds [ms]
unsigned long TimeOut = 400;

/*ROS2 Constants*/
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

/* ROS topics related variables*/
std_msgs__msg__String debugMsg;
geometry_msgs__msg__Twist ctrlCmdMsg;
rcl_publisher_t debugPublisher;
rcl_subscription_t ctrlCmdSubscription;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t ctrlCmdExecutor;
struct CtrlRequest* driveRequest; // DON'T use this variable dirctly, always use the getters and setters

float turnFactor = 0.5; 
int motor1, motor2;

// RC
unsigned long ch1_start_time = 0;
unsigned long ch2_start_time = 0;
int xRaw = 0;
int yRaw = 0;
int mode_switch;       // the current state of the button


// error function
void error_loop(){
  while(1){
    Serial.print("Micro ROS Error... \n");
  }
}

boolean isRCActive(){
  return (xRaw > MIN_T && xRaw < MAX_T && yRaw > MIN_T && yRaw < MAX_T);
  // return false;
}

/*Genarate debug String and push to the topic*/
void generate_debug_data() {
  int steering = getSteeringRequest(driveRequest);
  int speed = getDrivingSpeedRequest(driveRequest);
  int rc= (int)isRCActive();
  const char *variable_names[] = { "Steering Request", "Speed Request", "xRaw", "yRaw", "isRCActive()"};    // names of the variables
  int variable_values[] = {steering, speed, (int)xRaw, (int)yRaw, rc};  // values of the variables

  char final_string[256] = "";
  char buffer[128];
  
  for (int i = 0; i < 5; i++) {
    snprintf(buffer, sizeof(buffer), "%s: %d | ", variable_names[i], variable_values[i]);
    strcat(final_string, buffer);
  }

  snprintf(debugMsg.data.data, debugMsg.data.capacity, "[MICROROS]: %s", final_string);
  debugMsg.data.size = strlen(debugMsg.data.data);
  RCSOFTCHECK(rcl_publish(&debugPublisher, &debugMsg, NULL));
}

// ROS Callbacks
void ctrlCmdCallback(const void *msgin) {
  if(!isRCActive()){
    const geometry_msgs__msg__Twist *steering_input = (const geometry_msgs__msg__Twist *)msgin;

    float ROS_Steering_Command = steering_input->angular.z; // Assuming angular.z is used for steering angle
    float ROS_Speed_Command = steering_input->linear.x; // Assuming linear.x is used for speed

    // ROS Calculations
    // Slope and y-intercept for scale ROS steering angle command +0.45 - 0 - -0.45 [rad] to 0(left) - 50(middlepoint) - 100(right)
    // => ROS_Steering_Command*ROS_Steering_Command_Slope+ROS_Steering_Command_yIntercept => -0.45*-111+50 = 99.95 (-0.45 rad => Full Right ~= 100)
    int tempSteeringRequest = (ROS_Steering_Command * ROS_Steering_Command_Slope) + ROS_Steering_Command_yIntercept;
    setSteeringRequest(driveRequest, tempSteeringRequest);

    // Slope and y-intercept for scale ROS speed command -60 - 0 - +60 [m/s] to 0(full reverse) - 50(stop) - 100(full forward)
    // ROS_Speed_Command*ROS_Speed_Command_Slope+Speed_Command_yIntercept => 15*3.3+50 = 99.5 => Full Forward = 100)
    int tempSpeedRequest = ROS_Speed_Command * ROS_Speed_Command_Slope + ROS_Speed_Command_yIntercept;
    setDrivingSpeedRequest(driveRequest, tempSpeedRequest);

    PreviousTime = millis();
  }
}

void IRAM_ATTR CH1_interrupt() {
    if (digitalRead(CH1RCPin) == HIGH) {
        // Rising edge - start timing
        ch1_start_time = micros();
    } else {
        // Falling edge - calculate pulse width
        xRaw = micros() - ch1_start_time;
    }
}

void IRAM_ATTR CH2_interrupt() {
    if (digitalRead(CH2RCPin) == HIGH) {
        // Rising edge - start timing
        ch2_start_time = micros();
    } else {
        // Falling edge - calculate pulse width
        yRaw = micros() - ch2_start_time;
    }
}

void setup() {
  Serial.begin(115200);
 
  //pin initialising
  pinMode(CH1RCPin, INPUT);
  pinMode(CH2RCPin, INPUT);
  pinMode(Motor1DirPin, OUTPUT);
  pinMode(Motor2DirPin, OUTPUT);

  // Initialize motor PWM channels to zero to prevent erratic motor startup
  digitalWrite(Motor1SpeedPWMPin, LOW);
  digitalWrite(Motor1SpeedPWMPin, LOW);
  ledcWrite(0, 0);  // Stop Motor 1
  ledcWrite(1, 0);  // Stop Motor 2

  // Attach interrupts to pins
  attachInterrupt(digitalPinToInterrupt(CH1RCPin), CH1_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2RCPin), CH2_interrupt, CHANGE);
 
  //Initialising PWM on ESP32
  ledcSetup(0, FREQ, RESOLUTION); // Channel 0 for MotorSpeedPWM1
  ledcSetup(1, FREQ, RESOLUTION); // Channel 1 for MotorSpeedPWM2
  ledcAttachPin(Motor1SpeedPWMPin, 0); // Attach MotorSpeedPWM1 to channel 0
  ledcAttachPin(Motor2SpeedPWMPin, 1); // Attach MotorSpeedPWM2 to channel 1

  driveRequest = createCtrlRequest(Steering_Middlepoint, Driving_Speed_Middlepoint);

  /* ROS Initialize */
  // set_microros_wifi_transports("ssid", "password", "xxx.xxx.xxx.xxx", 8888); // microros over wifi
  set_microros_transports(); // microros over serial
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));// create node
  RCCHECK(rclc_publisher_init_best_effort(&debugPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/snower/debug")); // create debug publisher

  // Initialize the String message
  debugMsg.data.data = (char *)malloc(100 * sizeof(char)); // Allocate memory for the string
  debugMsg.data.size = 0;
  debugMsg.data.capacity = 100;

  // Create subscription
  RCCHECK(rclc_subscription_init_default(&ctrlCmdSubscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"/snower/ctrl_cmd"));

  // Initialize executor
  RCCHECK(rclc_executor_init(&ctrlCmdExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&ctrlCmdExecutor, &ctrlCmdSubscription, &ctrlCmdMsg, &ctrlCmdCallback, ON_NEW_DATA));

}

void getRC(){
  //X-axis control
  int angle = map(xRaw,993,2016,-255,255);
  int speed = map(yRaw,1027,2010,-255,255);
  setSteeringRequest(driveRequest, angle);
  setDrivingSpeedRequest(driveRequest, speed);
  PreviousTime = millis();

}

void driving() {

  int speed = getDrivingSpeedRequest(driveRequest);
  int angle = getSteeringRequest(driveRequest);
  
  int _x, _y;
  if(speed + (abs(angle) * turnFactor) > 255 || speed - (abs(angle) * turnFactor) < -255) {
    _x = angle * (255 / (abs(speed) + (abs(angle) * turnFactor)));
    _y = speed * (255 / (abs(speed) + (abs(angle) * turnFactor)));
    angle=_x;
    speed=_y;
  }
  
  motor1 = speed + (angle*turnFactor);
  motor2 = -speed + (angle*turnFactor);

  // Ensure motor PWM values are within limits
  motor1 = constrain(motor1, -255, 255);
  motor2 = constrain(motor2, -255, 255);

  digitalWrite(Motor1DirPin, motor1 >= 0);
  ledcWrite(0, abs(motor1)); // Writing PWM to channel 0 (AOUT1)
  digitalWrite(Motor2DirPin, motor2 >= 0);
  ledcWrite(1, abs(motor2)); // Writing PWM to channel 1 (MotorSpeedPWM2)

}

void loop() {
  delay(30);
  generate_debug_data();

  if(isRCActive()){
    getRC();
  }

  CurrentTime = millis();
  if ((CurrentTime - PreviousTime) >= TimeOut) {
    setSteeringRequest(driveRequest, 0);
    setDrivingSpeedRequest(driveRequest, 0);
  }


  driving();
  // Spin the executor to handle incoming messages
  rclc_executor_spin_some(&ctrlCmdExecutor, RCL_MS_TO_NS(100));

}