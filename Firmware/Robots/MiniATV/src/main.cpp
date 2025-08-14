/*
 *  NOTE! NOTE! NOTE! 
 *  DISCONNECT DRIVE MOTOR AND STEERING MOTOR
 *  OR POWER SUPPLY OF MOTORS 
 *  BEFORE PROGRAMMING ESP32!
 *  THERE CAN BE POTENTIALLY
 *  HAZARDOUS RISK/UNDESIRED MOTOR OUTPUT!!!
 */

#include "main.hpp"

float ROS_Speed_Command = 0.0;             // AckermannDriveStamped.drive.speed is float32 and speed is in m/s, Positive = Forward and Negative Backward (Reverse)
volatile float ROS_Speed_Measured = 0.0;   // Measured drive speed (send via ROSserial)is float32 and speed is in m/s, Positive = Forward and Negative Backward (Reverse) (volatile because use in interrupt)
float ROS_Steering_Command = 0.0;          // AckermannDriveStamped.drive.steering_angle is float32 and steering angle in radians, Positive = Left and Negative = Right
float ROS_Steering_Measured = 0.0;         // Measured steering angle (send via ROSserial) is float32 and steering angle in radians. Steering angle is real Wheel steering angle, not steppermotor angle! Positive = Left and Negative = Right.
unsigned long ROS_Control_Command_ID = 0;  // Current ROS input ID (sequence ID: consecutively increasing ID)
unsigned ROS_Missing_Packet_Count = 0;     // Counter to count Missing ROS subsequent packets
volatile boolean Safety_SW_State = 1;      // State for Safety Switch (volatile because use in interrupt)

/* Variables for Steering */
volatile long Steering_AD_Value = 2047;
volatile long Steering_Potentiometer = 50;     // store the value read (ADC)
volatile long Steering_Difference = 0;         // Difference between requested steering value and actual steering value (volatile because use in interrupt)
volatile boolean Steering_Limit_SW_State = 1;  // State for limit switch (volatile because use in interrupt)
volatile boolean Steering_Motor_Pulse = 0;     // Motor drive pulse (volatile because use in interrupt)
boolean Steering_Enable = 0;                   // Enabling or disabling steering
volatile boolean Steering_Direction;                    // '1' = left CW and '0' = right CCW
volatile int Half_Step_Count = 0;
volatile int Last_Potentiometer = 50;

/* Variables for Speed measurement and Odometry calculation */
volatile long FR_Wheel_Pulses = 0;        // Front Right Wheel Pulse count (volatile because use in interrupt)
volatile long FL_Wheel_Pulses = 0;        // Front Left Wheel Pulse count (volatile because use in interrupt)
volatile float Odometry = 0.0;            // Odometry value, calculated from front wheels in millimeters [mm]. NOTE! REVERSE DECREASE ODOMETRY (volatile because use in interrupt)
volatile float Previous_Odometry = 0.0;   // Previous Odometry value (volatile because use in interrupt)
const float Odometry_Coefficient = (Wheel_Circumference / Wheel_Pulse_Magnet_Count / 2);  // Odometry_Coefficient: Correlation between wheel pulse magnets and real tyre arc and divided by 2, because odometry is average from Left and Right wheels

/* Variables for Driving */
volatile boolean Driving_Direction;   // Driving Direction 0 = Reverse and 1 = Forward (volatile because use in interrupt)
long Driving_SpeedPWM_DutyCycle;      // PWM Duty Cycle for Driving motor controlle, 0 stop

/* Variables for RC "pwm reading" */
int RC_in[RC_input_Count];  // an array to store the calibrated input from receiver
boolean RC_Disable = 0;     // Disable RC control
unsigned long rc_time;      // store current time for RC measurement
unsigned long rc_update;    // previous time of RC measurement
int mode_switch;       // the current state of the button

/* Setting Driving PWM Properties */
const int Driving_PWMFreq = 1000;      // 1000Hz
const int Driving_PWMChannel = 0;      // Channel 0, 16 channels in total, 0-15
const int Driving_PWMResolution = 10;  // Resolution, the value is 0-20

/* Time variables */
unsigned long Current_Time = 0;   // Time now in milli seconds [ms]
unsigned long Previous_Time = 0;  // Last iteration time in milli seconds [ms]

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

/* Init ESP32 timers */
ESP32Timer Steering_Pulse_Timer(0);
ESP32Timer Speed_Calculation_Timer(1);
ESP32Timer Steering_Calculation_Timer(2);

// error function
void error_loop(){
  while(1){
    Serial.print("Micro ROS Error... \n");
  }
}


/*Genarate debug String and push to the topic*/
void error_debug(char error_cause[256]) {
  snprintf(debugMsg.data.data, debugMsg.data.capacity, "CRITICAL ERROR : %s", error_cause);
  debugMsg.data.size = strlen(debugMsg.data.data);
  RCSOFTCHECK(rcl_publish(&debugPublisher, &debugMsg, NULL));
}


// Driving subroutine
void Driving() {
  if (getDrivingSpeedRequest(driveRequest) > Driving_Speed_Middlepoint) {  // speedRequest >50 => Drive Forward
    Driving_Direction = Forward;
    Driving_SpeedPWM_DutyCycle = ((getDrivingSpeedRequest(driveRequest) - Driving_Speed_Middlepoint) * Driving_Speed_Duty_Coef);  // speedRequest 75 => Half Gas Forward => (75-50 = 25) 25*20 = DutyCyle 500 (1023 = MAX)
  } else if (getDrivingSpeedRequest(driveRequest) < Driving_Speed_Middlepoint) {                                                  // speedRequest <50 => Drive Reverse
    Driving_Direction = Backward;
    Driving_SpeedPWM_DutyCycle = ((Driving_Speed_Middlepoint - getDrivingSpeedRequest(driveRequest)) * Driving_Speed_Duty_Coef);  // speedRequest 25 => Half Gas Reverse => (50-25 = 25) 25*20 = DutyCyle 500 (1023 = MAX)
  } else {
    Driving_SpeedPWM_DutyCycle = 0;
  }
  ledcWrite(Driving_PWMChannel, Driving_SpeedPWM_DutyCycle);
  digitalWrite(DrivingDirPin, Driving_Direction);
}


// Safety Switch subroutine (interrupted)
void Safety_Switch() {
  Safety_SW_State = digitalRead(SafetySWPin);
  if(Safety_SW_State == 1){
    ledcWrite(Driving_PWMChannel, 0);
  }
}


// Limit Switch subroutine (interrupted)
void IRAM_ATTR Steering_Limit() {
  Steering_Limit_SW_State = digitalRead(SteeringLimitSWPin);
  /* if (Limit_SW_State == 0){
  Driving_Enable = 0;
   }                */
}


// Front Right Wheel Pulse Interrupt
void IRAM_ATTR Front_Right_Wheel_Pulse() {
  //FR_Wheel_Pulses = ++FR_Wheel_Pulses;
  if (Driving_Direction == Forward) {  // Forward
    FR_Wheel_Pulses++;
  } else {  // Reverse
    FR_Wheel_Pulses--;
  }
}


// Front Left Wheel Pulse Interrupt
void IRAM_ATTR Front_Left_Wheel_Pulse() {
  //FL_Wheel_Pulses = ++FL_Wheel_Pulses;
  if (Driving_Direction == Forward) {  // Forward
    FL_Wheel_Pulses++;
  } else {  // Reverse
    FL_Wheel_Pulses--;
  }
}


//Timer interrupt for Steering Pulse
bool IRAM_ATTR Steering_Pulse_Interrupt(void* param) {

  if (Steering_Enable == 1) {
    
    if(((-1 * Half_Step_Count)<Max_Half_Step_Count && Steering_Direction == Left) || 
    (Half_Step_Count<Max_Half_Step_Count && Steering_Direction == Right)){

      Steering_Motor_Pulse = !Steering_Motor_Pulse;
      
      if((Steering_Direction == Right && Steering_Potentiometer > Last_Potentiometer) ||
        (Steering_Direction == Left && Steering_Potentiometer < Last_Potentiometer) ){
        Last_Potentiometer = Steering_Potentiometer;
        Half_Step_Count = 0;
      }

      if(Steering_Direction == Right){
        Half_Step_Count++;
      } else {
        Half_Step_Count--;
      }

    }
  } else {
    Steering_Motor_Pulse = 0;
  }
  digitalWrite(SteeringPulsePin, Steering_Motor_Pulse);
  return true; // Return true to indicate the interrupt was handled
}


// Timer interrupt for Real Speed Calculation
bool IRAM_ATTR Speed_Calculation_Interrupt(void* param) {
  Previous_Odometry = Odometry;
  Odometry = (FL_Wheel_Pulses + FR_Wheel_Pulses) * Odometry_Coefficient;
  ROS_Speed_Measured = (Odometry - Previous_Odometry) / Speed_Calculation_Interval;  // (mm-mm)/ms = m/s
  // Speed_Measured = ROS_Speed_Measured * Speed_Measurement_Slope + Speed_Measurement_yIntercept;   // Convert -/+ m/s to scale 0-50-100
  return true; // Return true to indicate the interrupt was handled
}

bool IRAM_ATTR Steering_Calculation_Interrupt(void* param) {
  // Read Steering Potentiometer scale it 0-100%, Scale RC value to 0-100%, Check Switches
  Steering_AD_Value = analogRead(SteeringPotPin);               // read the potentiometer input pin
  Steering_Potentiometer = Steering_AD_Value * 100 / ADC_Bits;  // and convert it to 0-100%

  Steering_Difference = getSteeringRequest(driveRequest) - Steering_Potentiometer;
  Steering_Difference = abs(Steering_Difference);

  if ((Steering_Difference > Steering_Deadband) && (getSteeringRequest(driveRequest) < Steering_Potentiometer)) {  // Steering more left?
    Steering_Enable = 1;
    Steering_Direction = Left;
  } else if ((Steering_Difference > Steering_Deadband) && (getSteeringRequest(driveRequest) > Steering_Potentiometer)) {  // Steering more right?
    Steering_Enable = 1;
    Steering_Direction = Right;
  } else {  // Don't steer :)
    Steering_Enable = 0;
  }
  digitalWrite(SteeringDirPin, Steering_Direction);

  return true;
}

// ROS Callbacks
void ctrlCmdCallback(const void *msgin) {
  if(mode_switch == 0){
  const geometry_msgs__msg__Twist *steering_input = (const geometry_msgs__msg__Twist *)msgin;

    ROS_Steering_Command = steering_input->angular.z; // Assuming angular.z is used for steering angle
    ROS_Speed_Command = steering_input->linear.x; // Assuming linear.x is used for speed

    // ROS Calculations
    // Slope and y-intercept for scale ROS steering angle command +0.45 - 0 - -0.45 [rad] to 0(left) - 50(middlepoint) - 100(right)
    // => ROS_Steering_Command*ROS_Steering_Command_Slope+ROS_Steering_Command_yIntercept => -0.45*-111+50 = 99.95 (-0.45 rad => Full Right ~= 100)
    int tempSteeringRequest = (ROS_Steering_Command * ROS_Steering_Command_Slope) + ROS_Steering_Command_yIntercept;
    setSteeringRequest(driveRequest, tempSteeringRequest);

    // Slope and y-intercept for scale ROS speed command -60 - 0 - +60 [m/s] to 0(full reverse) - 50(stop) - 100(full forward)
    // ROS_Speed_Command*ROS_Speed_Command_Slope+Speed_Command_yIntercept => 15*3.3+50 = 99.5 => Full Forward = 100)
    int tempSpeedRequest = ROS_Speed_Command * ROS_Speed_Command_Slope + ROS_Speed_Command_yIntercept;
    setDrivingSpeedRequest(driveRequest, tempSpeedRequest);
  }
}


/*Genarate debug String and push to the topic*/
void generate_debug_data() {
  int steering = getSteeringRequest(driveRequest);
  int speed = getDrivingSpeedRequest(driveRequest);
  const char *variable_names[] = { "Steering Request", "Steering Potentiometer", "Speed Request", "step", "Safety" };    // names of the variables
  int variable_values[] = {steering,(int)Steering_Potentiometer, speed, (int)Half_Step_Count, (int)Safety_SW_State};  // values of the variables

  char final_string[256] = "";
  char buffer[128];
  
  for (int i = 0; i < 5; i++) {
    snprintf(buffer, sizeof(buffer), "%s: %d | ", variable_names[i], variable_values[i]);
    strcat(final_string, buffer);
  }

  snprintf(debugMsg.data.data, debugMsg.data.capacity, final_string);
  debugMsg.data.size = strlen(debugMsg.data.data);
  RCSOFTCHECK(rcl_publish(&debugPublisher, &debugMsg, NULL));
}



void setup() {
  Serial.begin(115200);

  pinMode(ModeSwitchPin, INPUT_PULLUP);                   // set ESP32 pin to input pull-up mode
  pinMode(SafetySWPin, INPUT);
  Safety_SW_State = digitalRead(SafetySWPin);
  attachInterrupt(digitalPinToInterrupt(SafetySWPin), Safety_Switch, CHANGE);
  pinMode(SteeringEnablePin, OUTPUT);
  digitalWrite(SteeringEnablePin, 0);
  pinMode(SteeringPulsePin, OUTPUT);
  pinMode(SteeringDirPin, OUTPUT);
  digitalWrite(SteeringDirPin, 1);
  pinMode(DrivingEnablePin, OUTPUT);
  digitalWrite(DrivingEnablePin, 0);
  pinMode(DrivingDirPin, OUTPUT);
  digitalWrite(DrivingDirPin, 1);

  pinMode(SteeringLimitSWPin, INPUT);
  Steering_Limit_SW_State = digitalRead(SteeringLimitSWPin);
  attachInterrupt(digitalPinToInterrupt(SteeringLimitSWPin), Steering_Limit, CHANGE);

  pinMode(FRWheelPulsePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(FRWheelPulsePin), Front_Right_Wheel_Pulse, RISING);

  pinMode(FLWheelPulsePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLWheelPulsePin), Front_Left_Wheel_Pulse, RISING);

  ledcSetup(Driving_PWMChannel, Driving_PWMFreq, Driving_PWMResolution);
  ledcAttachPin(DrivingSpeedPWMPin, Driving_PWMChannel);

  setup_pwmRead();  // call routine to setup RC input and interrupts

  // cal_steering_position();

  Steering_Pulse_Timer.attachInterruptInterval(Steering_Speed, Steering_Pulse_Interrupt);
  Speed_Calculation_Timer.attachInterruptInterval(Speed_Calculation_Interval * 1000, Speed_Calculation_Interrupt);
  Steering_Calculation_Timer.attachInterruptInterval(2*Steering_Speed, Steering_Calculation_Interrupt);

  pinMode(HWIsolatorEnablePin, OUTPUT);
  digitalWrite(HWIsolatorEnablePin, 1);

  driveRequest = createCtrlRequest(Steering_Middlepoint, Driving_Speed_Middlepoint);

  /* ROS Initialize */
  set_microros_serial_transports(Serial); // microros over serial
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));// create node
  RCCHECK(rclc_publisher_init_best_effort(&debugPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/atv/debug")); // create debug publisher

  // Initialize the String message
  debugMsg.data.data = (char *)malloc(100 * sizeof(char)); // Allocate memory for the string
  debugMsg.data.size = 0;
  debugMsg.data.capacity = 100;

  // Create subscription
  RCCHECK(rclc_subscription_init_default(
  &ctrlCmdSubscription,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "/atv/ctrl_cmd"));

  // Initialize executor
  RCCHECK(rclc_executor_init(&ctrlCmdExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&ctrlCmdExecutor, &ctrlCmdSubscription, &ctrlCmdMsg, &ctrlCmdCallback, ON_NEW_DATA));
}


void loop() {  
  delay(50); // to avoid the memory address CORRUPTED error and SW_CPU_RESET & SPI_FAST_FLASH_BOOT
  
  mode_switch = digitalRead(ModeSwitchPin);    // read new state 
  generate_debug_data();
  
  Current_Time = millis();
  if ((Current_Time - Previous_Time) >= ROS_Interval) {
    if (ROS_Missing_Packet_Count >= ROS_Max_Missing_Packets) {
      RC_Disable = 0;
      ROS_Steering_Command = 0;
      ROS_Speed_Command = 0;
    } else {
      ROS_Missing_Packet_Count = ++ROS_Missing_Packet_Count;
    }
    Previous_Time = Current_Time;
  } 
  
  if (mode_switch == 1) {
    rc_time = millis();
    if (RC_avail() || rc_time - rc_update > 25) {  // if RC data is available or 25ms has passed since last update (adjust to be equal or greater than the frame rate of receiver)
      rc_update = rc_time;
      //print_RCpwm();                                   // uncommment to print raw data from receiver to serial
      for (int i = 0; i < RC_input_Count; i++) {  // run through each RC channel
        RC_in[i] = RC_decode(i);                  // receiver channel and apply failsafe
      }
    }
    int tempSteeringRequest = (RC_in[0]-RC_Minimum)/RC_Scaler;   // Convert RC PWM value 1100 - 1900 to 0-100%, y = (x-1100)/8
    setSteeringRequest(driveRequest, tempSteeringRequest);

    int tempSpeedRequest = (RC_in[1] - RC_Minimum) / RC_Scaler;
    setDrivingSpeedRequest(driveRequest, tempSpeedRequest);
    
  }

  if (Safety_SW_State == 0) {
    Driving();
  }

  // Spin the executor to handle incoming messages
  rclc_executor_spin_some(&ctrlCmdExecutor, RCL_MS_TO_NS(100));


}