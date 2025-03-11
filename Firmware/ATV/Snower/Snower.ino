#include <stdio.h>
#include <ESP32TimerInterrupt.h>    // 2.3.0 version
#include <micro_ros_arduino.h>
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
#define PullUpSpeedLPin 12 //Left speed sensor
#define PullUpSpeedRPin 14 //Right speed sensor

//Constants
#define FREQ  490  //AnalogWrite frequency
#define MAX_T 2500 //Max signal threshold
#define MIN_T 500  //Min signal threshold
#define RESOLUTION 8 //PWM resolution (8-bit, range from 0-255)
#define LINEAR_X_DEFAULT 0   // Middle point of speed
#define ANGULAR_Z_DEFAULT 0  // middlepoint of angle
#define GENERAL_BLOCK_FREQUENCY 40   // Odometry publish rate in Hz
#define DEBUG_PUBLISHER_FREQUENCY 2  // Odometry publish rate in Hz
#define SPEED_PUBLISHER_FREQUENCY 5  // Odometry publish rate in Hz
#define PWM_SLOPE 340.00 // Slope of the linearization function (m)
#define PWM_INTERCEPT -17.00 // Intercept of the linearization function (C)
#define MIN_PWM 50 // Minimum PWM value to start the motors
#define MAX_PWM 255 // Maximum PWM value to start the motors
#define RC_PWM_UPPER_THRESHOLD 1600  // Upper threshold for RC PWM
#define RC_PWM_LOWER_THRESHOLD 1400  // Lower threshold for RC PWM

/* Time variables */
unsigned long PreviousTime = 0; // Last iteration time in milli seconds [ms]
unsigned long LastMCEnable = 0; // last enable motor controller time
unsigned long TimeOut = 200;  // control command time out
unsigned long MCTimeout = 60000;  // motor controller time out
unsigned long General_block_LET = 0; // General block last executed time
unsigned long debug_publisher_LET = 0; // General block last executed time
unsigned long speed_publisher_LET = 0; // General block last executed time

/*ROS2 Constants*/
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

/* ROS topics related variables*/
std_msgs__msg__String debugMsg;
std_msgs__msg__Float32 speedLeft;
std_msgs__msg__Float32 speedRight;
geometry_msgs__msg__Twist ctrlCmdMsg;
rcl_publisher_t debugPublisher;
rcl_publisher_t speedLeftPublisher;
rcl_publisher_t speedRightPublisher;
rcl_subscription_t ctrlCmdSubscription;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t ctrlCmdExecutor;

// Driving related variables
struct CommandVelocity* cmdVelDiffDrive; // DON'T use this variable dirctly, always use the getters and setters

// RC related variables
volatile unsigned long ch1_start_time = 0;
volatile unsigned long ch2_start_time = 0;
volatile int rc_z_pwm = 0;
volatile int rc_x_pwm = 0;

// Speed sensors
/* Left speed sensor*/
volatile unsigned long leftToothCount = 0;
volatile unsigned long leftLastEdgeTime = 0;
volatile unsigned long leftLastPublishTime = 0;
volatile int leftLastState = LOW;

/* Right speed sensor*/
volatile unsigned long rightToothCount = 0;
volatile unsigned long lastRightCount = 0;
volatile unsigned long rightLastEdgeTime = 0;
volatile unsigned long rightLastPublishTime = 0;
volatile int rightLastState = LOW;

/* Speed timing Constants*/
const unsigned long DEBOUNCE_THRESHOLD_US = 5;


void errorLoop() {
    static int retryCount = 0;
    retryCount++;

    // Disable motors as a safety measure
    digitalWrite(MCEnablePin, LOW);
    delay(2000);

    if (retryCount > 5) {
      Serial.println("[ERROR LOOP] Too many retries, halting or resetting system...");
      // force a system reset and try again
      ESP.restart(); 
    } 
    else {
      microrosInit();
    }
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

// TO DO: check if there is a better way than a "DigitalRead" in order to read the datapins.
//        Something like direct regiser access, or PCNT or RMT peripherals to handle pulse measurements in hardware
// left wheel speed 
void IRAM_ATTR speedLeft_interrupt(){
  int currentState = digitalRead(SpeedSensorLeftPin);
  
  if(currentState == HIGH) {
    if((millis() - leftLastEdgeTime) > DEBOUNCE_THRESHOLD_US) {
      leftToothCount++;
      leftLastEdgeTime = millis();
    }
  }
}

// right wheel speed 
void IRAM_ATTR speedRight_interrupt(){
  int currentState = digitalRead(SpeedSensorRightPin);
  
  if(currentState == HIGH && rightLastState == LOW) {
    if((millis() - rightLastEdgeTime) > DEBOUNCE_THRESHOLD_US) {
      rightToothCount++;
      rightLastEdgeTime = millis();
    }
  }
  rightLastState = currentState;
}

void IRAM_ATTR CH1_interrupt() {
    if (digitalRead(CH1RCPin) == HIGH) {
        // Rising edge - start timing
        ch1_start_time = micros();
    } else {
        // Falling edge - calculate pulse width
        rc_z_pwm = micros() - ch1_start_time;
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

void publishSpeedData() {
  unsigned long now = millis();
  unsigned long pulsesLeft = 0;
  unsigned long pulsesRight = 0;
  unsigned long timeDuration = 0;
  
  noInterrupts();
  timeDuration = millis() - leftLastPublishTime;
  pulsesLeft = leftToothCount;
  leftToothCount = 0;
  leftLastPublishTime = now;
  interrupts();
    
  float pulsesPerSecLeft = (pulsesLeft / (timeDuration/1000.00));
  speedLeft.data = pulsesPerSecLeft;
  RCSOFTCHECK(rcl_publish(&speedLeftPublisher, &speedLeft, NULL));
  
  noInterrupts();
  timeDuration = millis() - rightLastPublishTime;
  pulsesRight = rightToothCount;
  rightToothCount = 0;
  rightLastPublishTime = now;
  interrupts();
    
  float pulsesPerSecRight = (pulsesRight / (timeDuration/1000.00));
  speedRight.data = pulsesPerSecRight;
  RCSOFTCHECK(rcl_publish(&speedRightPublisher, &speedRight, NULL));

  // char final_string[128] = "";
  // snprintf(final_string, 128, "pulsesLeft: %d, pulsesRight: %d, pulsesPerSecLeft: %f, pulsesPerSecRight: %f", pulsesLeft, pulsesRight, pulsesPerSecLeft, pulsesPerSecRight);
  // debugDataPublisher(final_string);
}

void microrosInit(){
  //set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT); // microros over wifi
  set_microros_transports(); // microros over serial
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
  RCCHECK(rclc_subscription_init_default(&ctrlCmdSubscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"/cmd_vel"));

  // init publishers
  RCCHECK(rclc_publisher_init_best_effort(&debugPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/debug")); // create debug publisher
  RCCHECK(rclc_publisher_init_best_effort(&speedLeftPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"/speed/left")); // create left speed publisher
  RCCHECK(rclc_publisher_init_best_effort(&speedRightPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"/speed/right")); // create right speed publisher

  // Initialize executor
  RCCHECK(rclc_executor_init(&ctrlCmdExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&ctrlCmdExecutor, &ctrlCmdSubscription, &ctrlCmdMsg, &cmdVelCallback, ON_NEW_DATA));
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
  digitalWrite(PullUpSpeedLPin, HIGH);
  digitalWrite(PullUpSpeedRPin, HIGH);

  // Attach interrupts to pins
  attachInterrupt(digitalPinToInterrupt(CH1RCPin), CH1_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2RCPin), CH2_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SpeedSensorLeftPin), speedLeft_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SpeedSensorRightPin), speedRight_interrupt, CHANGE);

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
    double leftSpeed = getLeftSpeed(cmdVelDiffDrive);
    double rightSpeed = getRightSpeed(cmdVelDiffDrive);
    
    // get the PWM values for the motors
    int leftMotorPWM = getPWMbySpeed(abs(leftSpeed));
    int rightMotorPWM = getPWMbySpeed(abs(rightSpeed));
    
    // Set motor direction based on speed values
    digitalWrite(MotorLeftDirPin, leftSpeed >= 0.0);
    digitalWrite(MotorRightDirPin, rightSpeed <= 0.0); // The right motor is mounted in reverse, so its direction logic is inverted

    rightMotorPWM = int (rightMotorPWM * 0.94); // Right motor is more powerful than the left one, so we reduce its PWM value by 4%

    // Output PWM values to the motor controller
    ledcWrite(0, abs(leftMotorPWM));
    ledcWrite(1, abs(rightMotorPWM)); 

    speedLeft.data = leftSpeed;
    RCSOFTCHECK(rcl_publish(&speedLeftPublisher, &speedLeft, NULL));
    
    speedRight.data = rightSpeed;
    RCSOFTCHECK(rcl_publish(&speedRightPublisher, &speedRight, NULL));

    // publish speed sensor data
    unsigned long now = millis();
    if (now - debug_publisher_LET >= (1000 / DEBUG_PUBLISHER_FREQUENCY)) {
      debug_publisher_LET = millis();
      char final_string[128] = "";
      snprintf(final_string, 128, "X: %f m/s, Z: %f rad/s, L_Speed: %f m/s, R_Speed :%f m/s, L_PWM: %d , R_PWM: %d", getLinearX(cmdVelDiffDrive), getAngularZ(cmdVelDiffDrive), leftSpeed, rightSpeed, leftMotorPWM, rightMotorPWM);
      debugDataPublisher(final_string);
    }
}

void loop() {

  // General block of the loop
  unsigned long now = millis();
  if (now - General_block_LET >= (1000 / GENERAL_BLOCK_FREQUENCY)) {
    General_block_LET = millis();

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

  }

  // publish speed sensor data
  // now = millis();
  // if (now - speed_publisher_LET >= (1000 / SPEED_PUBLISHER_FREQUENCY)) {
  //   speed_publisher_LET = millis();
  //   publishSpeedData();
  // }

  // Spin the executor to handle incoming messages
  rclc_executor_spin_some(&ctrlCmdExecutor, RCL_MS_TO_NS(100));

}