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

#include <main.hpp>
 
/* Time variables */
unsigned long PreviousTime = 0; // Last iteration time in milli seconds [ms]
unsigned long LastMCEnable = 0; // last enable motor controller time
unsigned long TimeOut = 50;  // control command time out
unsigned long MCTimeout = 60000;  // motor controller time out
unsigned long General_block_LET = 0; // General block last executed time
unsigned long debug_publisher_LET = 0; // General block last executed time
unsigned long speed_publisher_LET = 0; // General block last executed time

/* ROS topics related variables*/
std_msgs__msg__String debugMsg;
std_msgs__msg__Float32 speedLeft;
std_msgs__msg__Float32 speedRight;
geometry_msgs__msg__Twist ctrlCmdMsg;
rcl_publisher_t debugPublisher;
rcl_publisher_t speedLeftPublisher;
rcl_publisher_t speedRightPublisher;
rcl_subscription_t ctrlCmdSubscription;
rcl_timer_t speedTimer;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t ctrlCmdExecutor;
 
// Driving related variables
struct CommandVelocity* cmdVelDiffDrive; // DON'T use this variable directly, always use the getters and setters
double cmdLeftSpeed;
double cmdRightSpeed;
 
// RC related variables
volatile unsigned long ch1_start_time = 0;
volatile unsigned long ch2_start_time = 0;
volatile int rc_z_pwm = 0;
volatile int rc_x_pwm = 0;
volatile long last_CH1_pulse_time = 0;
 
// Speed sensors
/* Left speed sensor*/
volatile unsigned long leftToothCount = 0;
volatile unsigned long leftLastEdgeTime = 0;
volatile int leftLastState = HIGH;
volatile float leftSpeed = 0.0;
int16_t countL = 0;
 
/* Right speed sensor*/
volatile unsigned long rightToothCount = 0;
volatile unsigned long rightLastEdgeTime = 0;
volatile int rightLastState = HIGH;
volatile float rightSpeed = 0;
int16_t countR = 0;
 
// Mutual exclusion to access speed related variables
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/* Speed timing Constants*/
const unsigned long DEBOUNCE_THRESHOLD_US = 5000; // microseconds
const float DISTANCE_TO_TOOTH_RATIO = 0.86355;  // cm/it
int errorRetryCount = 0;
int safeRetryCount = 0;
 
void errorLoop() {
  errorRetryCount++;
  digitalWrite(MCEnablePin, LOW); // Disable motors as a safety measure
  delay(1000);
 
  if (errorRetryCount > 5) {
    ESP.restart();
  }
  else {
    microrosCleanup();
    microrosInit();
  }
}  
 
bool safePublish(rcl_publisher_t* publisher, void* msg, const char* publisher_name) {
  rcl_ret_t rc = rcl_publish(publisher, msg, NULL);
  if (rc != RCL_RET_OK) {
    safeRetryCount++;
    delay(100);
    if (safeRetryCount > 3) {
      ESP.restart();
    }
    return false;
  }
  safeRetryCount = 0;  // Reset retry count on success
  return true;
}
 
boolean isRCActive(){
  return (rc_z_pwm > MIN_T && rc_z_pwm < MAX_T && rc_x_pwm > MIN_T && rc_x_pwm < MAX_T);
}
 
/*Genarate debug String and push to the topic*/
void debugDataPublisher(char final_string[128]) {
  snprintf(debugMsg.data.data, debugMsg.data.capacity, "[SNOWER]: %s", final_string);
  debugMsg.data.size = strlen(debugMsg.data.data);
  RCSOFTCHECK(rcl_publish(&debugPublisher, &debugMsg, NULL));
}
 
// ROS Callbacks
void cmdVelCallback(const void *msgin) {
  if(!isRCActive()){
    const geometry_msgs__msg__Twist *steering_input = (const geometry_msgs__msg__Twist *)msgin;
    setCmdVelDiffDrive(cmdVelDiffDrive, steering_input->linear.x, steering_input->angular.z);
    PreviousTime = millis();
  }
}

 
void IRAM_ATTR CH1_interrupt() {
    if (digitalRead(CH1RCPin) == HIGH) {
        // Rising edge - start timing
        ch1_start_time = micros();
    } else {
        // Falling edge - calculate pulse width
        rc_z_pwm = micros() - ch1_start_time;
        last_CH1_pulse_time = millis();
    }
}
 
void IRAM_ATTR CH2_interrupt() {
    if (digitalRead(CH2RCPin) == HIGH) {
        // Rising edge - start timing
        ch2_start_time = micros();
    } else {
        // Falling edge - calculate pulse width
        rc_x_pwm = micros() - ch2_start_time;
    }
}

void publishSpeed(rcl_timer_t * timer, int64_t last_call_time){
  static unsigned long LastPublishTime = 0;
  unsigned long currentTime = millis();
  unsigned long timeDuration;
    
  timeDuration = currentTime - LastPublishTime;
  LastPublishTime = currentTime;
  // portENTER_CRITICAL_ISR(&mux);
  // unsigned long pulsesLeft = leftToothCount;
  // leftToothCount = 0;
  // unsigned long pulsesRight = rightToothCount;
  // rightToothCount = 0;
  // portEXIT_CRITICAL_ISR(&mux);
  pcnt_get_counter_value(PCNT_LEFT_UNIT, &countL);
  pcnt_counter_clear(PCNT_LEFT_UNIT);
  pcnt_get_counter_value(PCNT_RIGHT_UNIT, &countR);
  pcnt_counter_clear(PCNT_RIGHT_UNIT);
 
  // leftSpeed = ((float)pulsesLeft) * DISTANCE_TO_TOOTH_RATIO * 10.0f / ((float)timeDuration);
  leftSpeed = ((float)countL) * DISTANCE_TO_TOOTH_RATIO * 10.0f / ((float)timeDuration);
  if(cmdLeftSpeed < 0.0){
    leftSpeed = -leftSpeed;
  } 
  // rightSpeed = ((float)pulsesRight) * DISTANCE_TO_TOOTH_RATIO * 10.0f / ((float)timeDuration);
  rightSpeed = ((float)countR) * DISTANCE_TO_TOOTH_RATIO * 10.0f / ((float)timeDuration);
  if(cmdRightSpeed < 0.0){
    rightSpeed = -rightSpeed;
  }
  
  speedLeft.data = leftSpeed;
  safePublish(&speedLeftPublisher, &speedLeft, "speedLeftPublisher");
  speedRight.data = rightSpeed;
  safePublish(&speedRightPublisher, &speedRight, "speedRightPublisher");
  
  char final_string[128] = "";
  // snprintf(final_string, 128, "L mes: %.2f, R mes: %.2f, L cmd: %.2f, R cmd: %.2f, L pls : %d, R pls: %d", leftSpeed, rightSpeed, cmdLeftSpeed, cmdRightSpeed, pulsesLeft, pulsesRight); // use this line to debug speed sensors
  //snprintf(final_string, 128, "L mes: %.2f, R mes: %.2f, L cmd: %.2f, R cmd: %.2f, L pls : %d, R pls: %d", leftSpeed, rightSpeed, cmdLeftSpeed, cmdRightSpeed, countL, countR); // use this line to debug speed sensors
  //snprintf(final_string, 128, "RC active : %d, rc_x_pwm : %d rc_z_pwm : %d", isRCActive(), rc_x_pwm, rc_z_pwm); // Use this line to debug/adjust RC
  //debugDataPublisher(final_string);
}


// Configure PCNT for falling edges detection
void setupPCNT(pcnt_unit_t unit, int pin) {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = pin,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_DIS,    // Ignore rising edges
    .neg_mode = PCNT_COUNT_INC,    // Increase counter on falling edges
    .counter_h_lim = 10000,        // high limit
    .counter_l_lim = 0,             // low limit
    .unit = unit,
    .channel = PCNT_CHANNEL_0
  };

  pcnt_unit_config(&pcnt_config);

  // Noise filtering (nanoseconds)
  pcnt_set_filter_value(unit, 1000);
  pcnt_filter_enable(unit);

  // start PCNT
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}
 
void microrosInit(){
  // in platformio.ini, set the board_microros_transport variable to wifi or serial depending on transport mode you want to use
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, agent_ip, agent_port); // microros over wifi
  //  set_microros_serial_transports(Serial); // microros over serial
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));// create node
 
  // Initialize the /debug String message
  debugMsg.data.data = (char *)malloc(128 * sizeof(char)); // Allocate memory for the string
  debugMsg.data.size = 0;
  debugMsg.data.capacity = 128;
  // Initialize the speed sensor msgs
  speedLeft.data = 0.00;
  speedRight.data = 0.00;
 
  // init subscribers
  RCCHECK(rclc_subscription_init_default(&ctrlCmdSubscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"/cmd_vel")); // use /cmd_vel_nav topic to disable smoothing and recovery
 
  // init publishers
  RCCHECK(rclc_publisher_init_best_effort(&debugPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/debug")); // create debug publisher
  RCCHECK(rclc_publisher_init_default(&speedLeftPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"/speed/left")); // create left speed publisher
  RCCHECK(rclc_publisher_init_default(&speedRightPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"/speed/right")); // create right speed publisher
 
  // init timer
  RCCHECK(rclc_timer_init_default(&speedTimer, &support, RCL_MS_TO_NS(1000/SPEED_PUBLISHER_FREQUENCY), publishSpeed));
 
  // Initialize executor
  RCCHECK(rclc_executor_init(&ctrlCmdExecutor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&ctrlCmdExecutor, &ctrlCmdSubscription, &ctrlCmdMsg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&ctrlCmdExecutor, &speedTimer));
}
 
void microrosCleanup(){
  rcl_ret_t rc;
  rc = rclc_executor_fini(&ctrlCmdExecutor);
  rc = rcl_publisher_fini(&debugPublisher, &node);
  rc = rcl_publisher_fini(&speedLeftPublisher, &node);
  rc = rcl_publisher_fini(&speedRightPublisher, &node);
  rc = rcl_subscription_fini(&ctrlCmdSubscription, &node);
  rc = rcl_node_fini(&node);
  rc = rclc_support_fini(&support);
  if (debugMsg.data.data != NULL) {
    free(debugMsg.data.data);
    debugMsg.data.data = NULL;
    debugMsg.data.size = 0;
    debugMsg.data.capacity = 0;
  }
}
 
void setup() {
  // delay(3000); // wait for Jetson to start the services
  Serial.begin(115200);
 
  //pin initialising
  pinMode(CH1RCPin, INPUT);
  pinMode(CH2RCPin, INPUT);
  pinMode(MotorLeftDirPin, OUTPUT);
  pinMode(MotorRightDirPin, OUTPUT);
  pinMode(MCEnablePin, OUTPUT);
  pinMode(SpeedSensorLeftPin, INPUT);
  pinMode(SpeedSensorRightPin, INPUT);
 
  //Initialising PWM on ESP32
  ledcSetup(0, FREQ, RESOLUTION); // Channel 0 for MotorSpeedPWM1
  ledcSetup(1, FREQ, RESOLUTION); // Channel 1 for MotorSpeedPWM2
  ledcAttachPin(MotorLeftPWMPin, 0); // Attach MotorSpeedPWM1 to channel 0
  ledcAttachPin(MotorRightPWMPin, 1); // Attach MotorSpeedPWM2 to channel 1
  
  // Initialize motor PWM channels to zero to prevent erratic motor startup
  digitalWrite(MotorLeftPWMPin, LOW);
  digitalWrite(MotorRightPWMPin, LOW);
  digitalWrite(MCEnablePin, LOW);
  ledcWrite(0, 0);  // Stop Motor 1
  ledcWrite(1, 0);  // Stop Motor 2
 
  // Attach interrupts to pins
  attachInterrupt(digitalPinToInterrupt(CH1RCPin), CH1_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2RCPin), CH2_interrupt, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(SpeedSensorLeftPin), speedLeft_interrupt, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(SpeedSensorRightPin), speedRight_interrupt, CHANGE);

  // Attach PCNT to speed sensors pins
  setupPCNT(PCNT_LEFT_UNIT, SpeedSensorLeftPin);
  setupPCNT(PCNT_RIGHT_UNIT, SpeedSensorRightPin);
 
  cmdVelDiffDrive = createCommandVelocity();
 
  microrosInit(); // microros initialize
}
 
double mapFloat(int x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 
void getRC(){
  //X-axis control
  //To do: Test out the rc controller for actual x and y values (min and max)
  
  double X = 0.0;
  double Z = 0.0;
 
  if(rc_x_pwm > RC_PWM_UPPER_THRESHOLD || rc_x_pwm < RC_PWM_LOWER_THRESHOLD){
    X = mapFloat(rc_x_pwm,1000,2000,-0.8,0.8);
  }
  if(rc_z_pwm > RC_PWM_UPPER_THRESHOLD || rc_z_pwm < RC_PWM_LOWER_THRESHOLD){
    Z = mapFloat(rc_z_pwm,1000,2000,-2.2,2.2);
  }
  setCmdVelDiffDrive(cmdVelDiffDrive, X, Z);
 
  // char final_string[128] = "";
  // snprintf(final_string, 128, "rc_x_pwm: %d, rc_z_pwm: %d, X: %f, Z: %f", rc_x_pwm, rc_z_pwm, X, Z);
  // debugDataPublisher(final_string);
 
  PreviousTime = millis();
 
}
 
void activateMotorController(){
  if(digitalRead(MCEnablePin) == LOW){
    digitalWrite(MCEnablePin, HIGH);
  }
  LastMCEnable = millis();
}
 
// Get PWM value by Speed (Linearization)
int getPWMbySpeed(double speed){
  int pwm = (int) ((PWM_SLOPE * speed) + PWM_INTERCEPT);
  if(pwm > MAX_PWM) {
      pwm = MAX_PWM;
  } else if (pwm < MIN_PWM) {
      pwm = 0;
  }
  return pwm;
}
 
 
void driving() {
 
  // activate motor controller using the relay switch
  if(getLinearX(cmdVelDiffDrive)!=0.0 || getAngularZ(cmdVelDiffDrive)!=0.0){
    activateMotorController();
  }
  // get differential speed values for the left and right motors
  cmdLeftSpeed = getLeftSpeed(cmdVelDiffDrive);
  cmdRightSpeed = getRightSpeed(cmdVelDiffDrive);
    
  // get the PWM values for the motors
  int leftMotorPWM = getPWMbySpeed(abs(cmdLeftSpeed));
  int rightMotorPWM = getPWMbySpeed(abs(cmdRightSpeed));
    
  // Set motor direction based on speed values
  digitalWrite(MotorLeftDirPin, cmdLeftSpeed >= 0.0);
  digitalWrite(MotorRightDirPin, cmdRightSpeed <= 0.0); // The right motor is mounted in reverse, so its direction logic is inverted
 
  leftMotorPWM = int (leftMotorPWM * 0.95); // left motor is more powerful than the right one, so we reduce its PWM value by 5%
 
  // Output PWM values to the motor controller
  ledcWrite(0, abs(leftMotorPWM));
  ledcWrite(1, abs(rightMotorPWM));
 
  // simulate odometry
  // speedLeft.data = cmdLeftSpeed;
  // safePublish(&speedLeftPublisher, &speedLeft, "speedLeftPublisher");
 
  //speedRight.data = cmdRightSpeed;
  //safePublish(&speedRightPublisher, &speedRight, "speedRightPublisher");
 
}
 
void loop() {
 
  // General block of the loop
  unsigned long now = millis();
  if (now - General_block_LET >= (1000 / GENERAL_BLOCK_FREQUENCY)) {
 

    // reset when RC is turned off (RC not active)
    if (last_CH1_pulse_time < General_block_LET) {
    rc_z_pwm = 0; // Reset if PWM on CH1 not active between two general block execution
    }
    
    // is Remote Controller active
    if(isRCActive()){
      getRC();
    }
  
    // cmd_vel timeout
    now = millis();
    if ((now - PreviousTime) >= TimeOut) {
      setCmdVelDiffDrive(cmdVelDiffDrive, 0.0, 0.0);
    }
 
    // Motor Control idle mode
    now = millis();
    if ((now - LastMCEnable) >= MCTimeout) {
      digitalWrite(MCEnablePin, LOW);
    }
    // main driving function
    driving();

    General_block_LET = millis();
  }

  // Spin the executor to handle incoming messages
  rclc_executor_spin_some(&ctrlCmdExecutor, RCL_MS_TO_NS(100));
 
}