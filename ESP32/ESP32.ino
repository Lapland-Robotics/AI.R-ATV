/*  06.2021
 *  ROS Serial
 *  Use teb_local_planner ROS package , ackermann_msgs/AckermannDriveStamped.msg
 *  Convert Radio Controler PWM for/to Stepper Motor Driver and ESC
 *  Potentiometer feed back for Steering
 *  Hall-sensor limit Switch
 *  and Safety Switch
 */

/*
 *  NOTE! NOTE! NOTE! 
 *  DISCONNECT DRIVE MOTOR AND STEERING MOTOR
 *  OR POWER SUPPLY OF MOTORS 
 *  BEFORE PROGRAMMING ESP32!
 *  THERE CAN BE POTENTIALLY
 *  HAZARDOUS RISK/UNDESIRED MOTOR OUTPUT!!!
 */
 
//  ros SIMULATOR WITHOUT REAL INTERFACE TO ATV
//#define Simulation                        // Compiler directives for simulation or not

 /* TODO:
  - LEDS?
  - Inclinometers (SPI) ?
  */

#include "ATV.h"

// ROS
// ROS Headers for ROSserial
#include <ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
// ROS Node Hande
ros::NodeHandle nh;                                    // Node Handle "nh"
// ROS In
float ROS_Steering_Command = 0.0;                      // AckermannDriveStamped.drive.steering_angle is float32 and steering angle in radians, Positive = Left and Negative = Right
                                                       // Note! here Steering angle is real Wheel steering angle, not steppermotor angle!
float ROS_Speed_Command = 0.0;                           // AckermannDriveStamped.drive.speed is float32 and speed is in m/s, Positive = Forward and Negative Backward (Reverse)
// ROS Out
float ROS_Steering_Measured = 0.0;                     // Measured steering angle (send via ROSserial) is float32 and steering angle in radians, Positive = Left and Negative = Right
                                                       // Note! here Steering angle is real Wheel steering angle, not steppermotor angle!
volatile float ROS_Speed_Measured = 0.0;               // Measured drive speed (send via ROSserial)is float32 and speed is in m/s, Positive = Forward and Negative Backward (Reverse) (volatile because use in interrupt)
ackermann_msgs::AckermannDriveStamped ROS_ControlState; // ROS_ControlState (ackermann message) = Measures Steering Angle(float32, radians), Measured Speed (m/s)
std_msgs::String debug_string;
#define ROS_Interval 200                               // ROS Commands update interval in milli second [ms]
//#define ROS_Interval 1000  // FOR DEBUGING VIA SERIAL PORT  
unsigned long ROS_Control_Command_ID = 0;              // Current ROS input ID (sequence ID: consecutively increasing ID)
unsigned ROS_Missing_Packet_Count = 0;                 // Counter to count Missing ROS subsequent packets
#define ROS_Max_Missing_Packets 10                     // How many (ROS_Interval) subsequent ROS command not receive and then reject ROS control, If ROS_Interval = 100 ms and ROS_Max_Missing_Packets = 10 than Max silent time is 100 ms * 10 = 1s 


// HW timers
// --> https://www.arduino.cc/reference/en/libraries/esp32timerinterrupt/ -->
// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#include "ESP32TimerInterrupt.h"
// Init ESP32 timers
ESP32Timer Steering_Pulse_Timer(0);
ESP32Timer Speed_Calculation_Timer(1);

// Time variables and constants
unsigned long Current_Time = 0;                 // Time now in milli seconds [ms]
unsigned long Previous_Time = 0;                // Last iteration time in milli seconds [ms]             

// Setting Driving PWM Properties 
const int Driving_PWMFreq = 1000;               // 1000Hz
const int Driving_PWMChannel = 0;               // Channel 0, 16 channels in total, 0-15
const int Driving_PWMResolution = 10;           // Resolution, the value is 0-20
// NOTE! NOTE!: The maximum value of duty is 2^resolution-1, 2^10-1 = 1023
#define DrivingPWMMaxDutyCycle 1023
#define Driving_Speed_Duty_Coef 20				// Dummy engineer Coefficient for scale PWM duty cycle 


// Pins (* Not in use in this context):
// GPIO36 = IN Potentiometer for steering position (from buffer amp)
// GPIO39 = *IN Reserved Analog In to future use (from buffer amp)
// GPIO34 = IN RC PCM input for Steering (defined in file pwmread_rcfailsafe) (Isolated)
// GPIO35 = IN RC PCM input for Driving (defined in file pwmread_rcfailsafe) (Isolated)
// GPIO32 = IN Safety Switch (Isolated)
// GPIO33 = IN Steering Limit switch (Isolated)
// GPIO25 = OUT Driving Direction (Isolated)
// GPIO26 = OUT Driving PWM Output (Isolated)
// GPIO27 = *OUT Reserved for Driving Enable (Isolated)
// GPIO14 = *IN Reserved for Driving Status from controlle (Isolated)
// GPIO12 = OUT Steering Direction (Isolated)
// GPIO13 = OUT Steering Pulse (Isolated)
// GPIO09 = OUT Steering Enable (Isolated)
// GPIO10 = *IN Reserved for Steering Status from motor controlle (Isolated)
// GPIO01 = OUT UART0 TX (Non isolated)
// GPIO03 = IN UART0 RX (Non isolated)
// GPIO22 = *OUT Reserved for I2C SCL (Non isolated)
// GPIO21 = *IN/OUT Reserved for I2C SDA (Non isolated)
// GPIO19 = *IN SPI MISO (Isolated)
// GPIO23 = *OUT SPI MOSI (Isolated)
// GPIO18 = *OUT SPI SCL (Isolated)
// GPIO05 = *OUT SPI CS0 (Isolated)
// GPIO17 = *OUT SPI CS1 (Isolated)
// GPIO16 = *OUT SPI CS2 (Isolated)
// GPIO04 = IN Front Left Wheel rotation pulse (Isolated)
// GPIO15 = IN Front Right Wheel rotation pulse (Isolated)
// GPIO02 = OUT Enable INput/OUTput Isolator IC's on PCB (Pulled Down on Boot)
// GPIO00 = *IN Button on ESP32_DevKit
// GPIO05 - GPIO08 and GPIO11 = *IN/OUT wired to pinheader (Non isolated)

// Constant for Hardware control
#define HWIsolatorEnablePin 2         // Enable or disable Isolator IC's on PCB

// Constants and Variables for RC "pwm reading"
unsigned long rc_time;                // store current time for RC measurement
unsigned long rc_update;              // previous time of RC measurement
//const int channels = 2;             // specify the number of receiver channels
int RC_in[RC_input_Count];            // an array to store the calibrated input from receiver
boolean RC_Disable = 0;               // Disable RC control
#define RC_Minimum 1000               // RC minimum is 1000us
#define RC_Scaler 10                  // Dummy engineer divider for scale RC signal to 0(neg max) - 50(middlepoint) - 100(pos max) => (RC_input - RC_min)/RC_Scaler => (1500-1000)/10 = 50

// Safety HW
#define SafetySWPin 32                  // If '0' = safe
volatile boolean Safety_SW_State = 1;   // State for Safety Switch (volatile because use in interrupt)

// Constants and Variables for Steering
#define SteeringPotPin 36               // Potentiometer wiper (middle terminal) connected to analog pin 0
#define SteeringDirPin 12               // Steering Motor Direction
#define SteeringPulsePin 13             // Steering Motor drive pulse
#define SteeringEnablePin 9             // Steering Motor Enable
                                        // Outside leads to ground and +5V
long Steering_AD_Value = 2047;
long Steering_Potentiometer = 50;       // Variable to store the value read (ADC)
long Steering_Request = 50;             // Requested steering value 0-100, 0 = Full Left, 50 = Center and 100 = Full Right
int Steering_Precision = 2;
volatile long Steering_Difference = 0;  // Difference between requested steering value and actual steering value (volatile because use in interrupt)
#define Steering_Deadband 2             // Acceptable steering error (here named "deadband"), to avoid steering jerking (bad steering position measurement and poor stepper motor drive)
#define ADC_Bits 4095;                  // ADC bits
#define Steering_Middlepoint 50			    // Steering Command Middle point
#define Steering_Left_Limit 2           // Left direction limit value for Steering Pot
#define Steering_Right_Limit 98         // Right direction limit value for Steering Pot
boolean Steering_Direction;             // '1' = left CW and '0' = right CCW
volatile boolean Steering_Limit_SW_State = 1;  // State for limit switch (volatile because use in interrupt)
volatile boolean Steering_Motor_Pulse = 0;     // Motor drive pulse (volatile because use in interrupt)
#define Steering_Speed_Fast 900         // Change Steering Speed Fast (half pulse 500 => 2*500 = 1000) 1000us ~ 1000Hz 
#define Steering_Speed_Slow 1200        // Change Steering Speed Slow (half pulse 3350 => 2*3350 = 6700)6700us ~ 150Hz
#define Steering_Speed_Change 10        // Steering difference when change Steering Speed from Fast to Slow and vica verse
#define SteeringLimitSWPin 33           // Hall-sensor input for Limit steering angle
boolean Steering_Enable = 0;            // Enabling or disabling steering
#define Left 1							            // Left = 1 constant
#define Right 0							            // Right = 0 constant

// Constants and Variables for Driving
#define DrivingDirPin 25              // Direction output for Driver motor controlle
#define DrivingSpeedPWMPin 26         // Speed output for Driver motor controller
#define DrivingEnablePin 27           // Driving Motor Controller enable (NOT IN USE)
volatile boolean Driving_Enable = 0;  // Enabling or disabling Driving (volatile because use in interrupt)
volatile boolean Driving_Direction;   // Driving Direction 0 = Reverse and 1 = Forward (volatile because use in interrupt)
long Driving_Speed_Request = 50;      // Requested Driving Speed 0-100, 0 = Full Reverse, 0 = Stop and 100 = Full Forward
long Driving_SpeedPWM_DutyCycle;      // PWM Duty Cycle for Driving motor controlle, 0 stop
#define Driving_Speed_Middlepoint 50  // Dummy engineering constant for setting middlepoint of Speed Command
#define Driving_Reverse_Limit 30      // Reverse Driving Speed limit (not actual speed m/s)
#define Driving_Forward_Limit 70      // Forward Driving Speed limit (not actual speed m/s)
#define Forward 1                     // Forward = 1
#define Backward 0                    // Backward (Reverse) = 0

// Constants and Variables for Speed measurement and Odometry calculation
#define FRWheelPulsePin  15              // Front Right Wheel rotation pulse, Use Hall Sensor with 8 magnets
#define FLWheelPulsePin  4               // Front Left Wheel rotation pulse, Use Hall Sensor with 8 magnets
#define Wheel_Circumference 900.0        // Wheels circumference in milli meters [mm]
#define Wheel_Pulse_Magnet_Count 16      // Magnet count in one wheel for measuring wheel pulses
volatile long FR_Wheel_Pulses = 0;       // Front Right Wheel Pulse count (volatile because use in interrupt)
volatile long FL_Wheel_Pulses = 0;       // Front Left Wheel Pulse count (volatile because use in interrupt)
volatile float Odometry = 0.0;           // Odometry value, calculated from front wheels in millimeters [mm]. NOTE! REVERSE DECREASE ODOMETRY (volatile because use in interrupt)
volatile float Previous_Odometry = 0.0;  // Previous Odometry value (volatile because use in interrupt)
//long Speed_Measured = 50;              // Measured Driving Speed 0-100, 0 = Full Reverse, 0 = Stop and 100 = Full Forward
#define Speed_Calculation_Interval 200.0 // Speed Calculation Interval in milli second [ms]
// Dummy engineering constants for setting Slope and y-intercept for calculation: Real Speed(ROS_Speed_Measured) * Slope + y-intercept
#define Speed_Measurement_Slope  12.0    // Example 3m/s*12+50 = 86
#define Speed_Measurement_yIntercept  50.0


// ROS Calculations
// Note! here Steering angle is real Wheel steering angle, not steppermotor angle!
// Slope and y-intercept for scale ROS steering angle command +0.45 - 0 - -0.45 [rad] to 0(left) - 50(middlepoint) - 100(right) 
// => ROS_Steering_Command*ROS_Steering_Command_Slope+ROS_Steering_Command_yIntercept => -0.45*-111+50 = 99.95 (-0.45 rad => Full Right ~= 100)
#define ROS_Steering_Command_Slope -111.0  
#define ROS_Steering_Command_yIntercept 50.0 
// Slope and y-intercept for scale ROS speed command -60 - 0 - +60 [m/s] to 0(full reverse) - 50(stop) - 100(full forward) 
// => ROS_Speed_Command*ROS_Speed_Command_Slope+Speed_Command_yIntercept => 15*3.3+50 = 99.5 => Full Forward = 100)
#define ROS_Speed_Command_Slope 64 //53.0 //3.3  
#define ROS_Speed_Command_yIntercept 50.0 
// Odometry_Coefficient: Correlation between wheel pulse magnets and real tyre arc and divided by 2, because odometry is average from Left and Right wheels
const float Odometry_Coefficient = (Wheel_Circumference/Wheel_Pulse_Magnet_Count/2);

/*
//Define Variables for PIDs
double SteeringPID_Input = Steering_Middlepoint;
double SteeringPID_Output;
double SteeringPID_Setpoint = Steering_Middlepoint;
double SpeedPID_Input = Driving_Speed_Middlepoint;
double SpeedPID_Output;
double SpeedPID_Setpoint = Driving_Speed_Middlepoint;
PID SteeringPID(&SteeringPID_Input, &SteeringPID_Output, &SteeringPID_Setpoint,2,5,1, DIRECT);
PID SpeedPID(&SpeedPID_Input, &SpeedPID_Output, &SpeedPID_Setpoint,2,5,1, DIRECT);
*/

// Steering subroutine
void Steering () {
  if (Steering_Request < Steering_Potentiometer){             // Steering more left?
    // Left limit activated or left pot limit?
	  if (Steering_Limit_SW_State == 0 && Steering_Potentiometer < Steering_Middlepoint || Steering_Potentiometer <= Steering_Left_Limit){
      Steering_Enable = 0;                                    // Limits on
      Steering_Request = Steering_Left_Limit;                 // Update value only for debubing presentation
    } else {                                                  // No limits, lets steer more to left
      Steering_Enable = 1;
      Steering_Direction = Left;
    }
    // Right limit activated or right pot limit?
  } else if (Steering_Request > Steering_Potentiometer){      // Steering more right?
    if (Steering_Limit_SW_State == 0 && Steering_Potentiometer > Steering_Middlepoint || Steering_Potentiometer >= Steering_Right_Limit){
      Steering_Enable = 0;                                    // Limits on
      Steering_Request = Steering_Right_Limit;                // Update value only for debubing presentation
    } else {                                                  // No limits, lets steer more to right
     Steering_Enable = 1;
     Steering_Direction = Right;
    }    
  } else {                                                    // Don't steer :)
    Steering_Enable = 0;
  }
  digitalWrite(SteeringDirPin, Steering_Direction);
}

// Driving subroutine
void Driving () {
  if (Driving_Enable == 1){
    if (Driving_Speed_Request > Driving_Speed_Middlepoint){                                                      // Driving_Speed_Request >50 => Drive Forward
      Driving_Direction = Forward;
      if (Driving_Speed_Request >= Driving_Forward_Limit){
        Driving_Speed_Request = Driving_Forward_Limit;
      }
      Driving_SpeedPWM_DutyCycle = ((Driving_Speed_Request-Driving_Speed_Middlepoint)*Driving_Speed_Duty_Coef);    // Driving_Speed_Request 75 => Half Gas Forward => (75-50 = 25) 25*20 = DutyCyle 500 (1023 = MAX)
    } else if (Driving_Speed_Request < Driving_Speed_Middlepoint){                                               // Driving_Speed_Request <50 => Drive Reverse
      Driving_Direction = Backward;
      if (Driving_Speed_Request <= Driving_Reverse_Limit){
        Driving_Speed_Request = Driving_Reverse_Limit;
      }
      Driving_SpeedPWM_DutyCycle = ((Driving_Speed_Middlepoint-Driving_Speed_Request)*Driving_Speed_Duty_Coef);    // Driving_Speed_Request 25 => Half Gas Reverse => (50-25 = 25) 25*20 = DutyCyle 500 (1023 = MAX)
    } else {
    Driving_SpeedPWM_DutyCycle = 0;
  }    
  ledcWrite(Driving_PWMChannel, Driving_SpeedPWM_DutyCycle);
 }
 digitalWrite(DrivingDirPin, Driving_Direction);
}
// Safety Switch subroutine (interrupted)
void Safety_Switch(){
  Safety_SW_State = digitalRead(SafetySWPin);
  /* if (Safety_SW_State == 0){
  Steering_Enable = 0;
  Driving_Enable = 0;
   }                */ 
}
// Limit Switch subroutine (interrupted)
void IRAM_ATTR Steering_Limit(){
  Steering_Limit_SW_State = digitalRead(SteeringLimitSWPin);
/* if (Limit_SW_State == 0){
  Driving_Enable = 0;
   }                */ 
}
// Front Right Wheel Pulse Interrupt
void IRAM_ATTR Front_Right_Wheel_Pulse(){
  //FR_Wheel_Pulses = ++FR_Wheel_Pulses;
  if (Driving_Direction == Forward){            // Forward
    FR_Wheel_Pulses++;
  } else {                                      // Reverse
    FR_Wheel_Pulses--;
  }
}
// Front Left Wheel Pulse Interrupt
void IRAM_ATTR Front_Left_Wheel_Pulse(){
  //FL_Wheel_Pulses = ++FL_Wheel_Pulses;
  if (Driving_Direction == Forward){            // Forward
    FL_Wheel_Pulses++;
  } else {                                      // Reverse
    FL_Wheel_Pulses--;
  }
}
//Timer interrupt for Steering Pulse
void IRAM_ATTR Steering_Pulse_Interrupt(void){                 
  if (Steering_Enable == 1){
   if (Steering_Difference <= Steering_Speed_Change){
     //timerAlarmWrite(Steering_Pulse_Timer, Steering_Speed_Slow, true);
     Steering_Pulse_Timer.setInterval(Steering_Speed_Slow, Steering_Pulse_Interrupt);
   } else {
     //timerAlarmWrite(Steering_Pulse_Timer, Steering_Speed_Fast, true);
     Steering_Pulse_Timer.setInterval(Steering_Speed_Fast, Steering_Pulse_Interrupt);
   }
  Steering_Motor_Pulse = !Steering_Motor_Pulse;
 } else {
  Steering_Motor_Pulse = 0;
 }
  digitalWrite(SteeringPulsePin, Steering_Motor_Pulse);
}

// Timer interrupt for Real Speed Calculation
void IRAM_ATTR Speed_Calculation_Interrupt (void){
  Previous_Odometry = Odometry;
  Odometry = (FL_Wheel_Pulses + FR_Wheel_Pulses)*Odometry_Coefficient;
  ROS_Speed_Measured = (Odometry - Previous_Odometry)/Speed_Calculation_Interval;             // (mm-mm)/ms = m/s
  //Speed_Measured = ROS_Speed_Measured*Speed_Measurement_Slope+Speed_Measurement_yIntercept;   // Conver -/+ m/s to scale 0-50-100 
}  

// ROS Callbacks
void cb_ROS_ControlCommand(const ackermann_msgs::AckermannDriveStamped & ackermann_input){
  if (ackermann_input.header.seq != ROS_Control_Command_ID){
    // Ackermann commands to variables
    ROS_Control_Command_ID = ackermann_input.header.seq;
    ROS_Steering_Command = ackermann_input.drive.steering_angle;
	  ROS_Speed_Command = ackermann_input.drive.speed;
	  Steering_Request = ROS_Steering_Command*ROS_Steering_Command_Slope+ROS_Steering_Command_yIntercept;
    Driving_Speed_Request = ROS_Speed_Command*ROS_Speed_Command_Slope+ROS_Speed_Command_yIntercept;
    ROS_Missing_Packet_Count = 0;
    RC_Disable = 1;
  } 
}

// ROS Subscribers
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> Control("/ackermann_cmd", &cb_ROS_ControlCommand);   // Control message for Steering and Driving control
// ROS Publishers
ros::Publisher State("atv_state", &ROS_ControlState);    // ATV State Publisher
ros::Publisher Debug("debug", &debug_string);    // ATV State Publisher


void setup() {
  //Serial.begin(57600);
  
  pinMode (SafetySWPin, INPUT);
  Safety_SW_State = digitalRead(SafetySWPin);
  attachInterrupt(digitalPinToInterrupt(SafetySWPin), Safety_Switch, CHANGE);
  pinMode (SteeringEnablePin, OUTPUT);
  digitalWrite (SteeringEnablePin, 0);
  pinMode (SteeringPulsePin, OUTPUT);
  pinMode (SteeringDirPin, OUTPUT);
  digitalWrite (SteeringDirPin, 1);
  pinMode (DrivingEnablePin, OUTPUT);
  digitalWrite (DrivingEnablePin, 0);
  pinMode (DrivingDirPin, OUTPUT);
  digitalWrite (DrivingDirPin, 1);

  pinMode(SteeringLimitSWPin, INPUT);
  Steering_Limit_SW_State = digitalRead(SteeringLimitSWPin);
  attachInterrupt(digitalPinToInterrupt(SteeringLimitSWPin), Steering_Limit, CHANGE);

  pinMode(FRWheelPulsePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(FRWheelPulsePin), Front_Right_Wheel_Pulse, RISING);
  
  pinMode(FLWheelPulsePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLWheelPulsePin), Front_Left_Wheel_Pulse, RISING);

  ledcSetup(Driving_PWMChannel, Driving_PWMFreq, Driving_PWMResolution);
  ledcAttachPin(DrivingSpeedPWMPin, Driving_PWMChannel);
  
  setup_pwmRead();                          // call routine to setup RC input and interrupts            

  Steering_Pulse_Timer.attachInterruptInterval(Steering_Speed_Slow, Steering_Pulse_Interrupt);
  Speed_Calculation_Timer.attachInterruptInterval(Speed_Calculation_Interval*1000, Speed_Calculation_Interrupt);
/*
  SteeringPID.SetOutputLimits(0,100);
  SpeedPID.SetOutputLimits(0,100);
  SteeringPID.SetSampleTime(10);
  SpeedPID.SetSampleTime(10);
  SteeringPID.SetMode(AUTOMATIC);     // Turn PIDs ON
  SpeedPID.SetMode(AUTOMATIC); 
*/  
  pinMode (HWIsolatorEnablePin, OUTPUT);
  digitalWrite (HWIsolatorEnablePin, 1);
  
  // IF STEERING CALIBRATION THAN PLACE IT HERE

  // ROS Initialize
  nh.initNode(); 
  nh.subscribe( Control );
  nh.advertise( State );
  nh.advertise( Debug );
}


void loop() {
debug_string.data = "Arctic AI & Robotics";
Debug.publish(&debug_string);
nh.spinOnce();

Current_Time = millis();
  if ((Current_Time - Previous_Time) >= ROS_Interval){
    if (ROS_Missing_Packet_Count >= ROS_Max_Missing_Packets){
      RC_Disable = 0;
      ROS_Steering_Command = 0;
      ROS_Speed_Command = 0;
    } else {
      ROS_Missing_Packet_Count = ++ROS_Missing_Packet_Count;
    }
    Previous_Time = Current_Time;

#ifdef Simulation                           // Compiler directives for simulation or not
    ROS_ControlState.drive.steering_angle = ROS_Steering_Command;
    ROS_ControlState.drive.speed = ROS_Speed_Command;
#else                                       // Simulation
    ROS_ControlState.drive.steering_angle = ROS_Steering_Measured;
    ROS_ControlState.drive.speed = ROS_Speed_Measured;
#endif
    State.publish(&ROS_ControlState);
	  nh.spinOnce();
   
/*    Serial.print ('\n');
    Serial.print ("RC_Disable: ");
    Serial.print (RC_Disable);
    Serial.print (" , ");
    Serial.print ("Steering_Difference: ");
    Serial.print (Steering_Difference);
    Serial.print (" , ");
    Serial.print ("Steering_Potentiometer: ");
    Serial.print (Steering_Potentiometer);
    Serial.print (" , ");
    Serial.print ("Steering_Request: ");
    Serial.print (Steering_Request);
    Serial.print (" , ");
/*    Serial.print ("SteeringPID_Output: ");
    Serial.print (SteeringPID_Output);
    Serial.print (" , "); *//*
    Serial.print ("Driving_Speed_Request: ");
    Serial.print (Driving_Speed_Request);
    Serial.print (" , ");
    Serial.print ("Speed_Measured: ");
    Serial.print (ROS_Speed_Measured);
    Serial.print (" , ");
/*    Serial.print ("SpeedPID_Output: ");
    Serial.print (SpeedPID_Output);
    Serial.print (" , "); *//*
    if (Steering_Limit_SW_State == 0){
      Serial.println("Steering_LimitSW_ON"); 
    } else {
      Serial.println("Steering_LimitSW_OFF");
    }
    if (Safety_SW_State == 0){
      Serial.println("Safety_LimitSW_ON"); 
    } else {
      Serial.println("Safety_LimitSW_OFF");
    }*/
  }

  if (RC_Disable == 0){
    rc_time = millis();
   if(RC_avail() || rc_time - rc_update > 25){           // if RC data is available or 25ms has passed since last update (adjust to be equal or greater than the frame rate of receiver)
      rc_update = rc_time;                           
      //print_RCpwm();                                   // uncommment to print raw data from receiver to serial
      for (int i = 0; i < RC_input_Count; i++){          // run through each RC channel
        RC_in[i] = RC_decode(i);                         // receiver channel and apply failsafe  
      }
    }
   // Steering_Request = (RC_in[0]-RC_Minimum)/RC_Scaler;   // Convert RC PWM value 1100 - 1900 to 0-100%, y = (x-1100)/8
   Driving_Speed_Request = (RC_in[1]-RC_Minimum)/RC_Scaler;
  }

// Read Steering Potentiometer scale it 0-100%, Scale RC value to 0-100%, Check Switches
Steering_AD_Value = analogRead(SteeringPotPin);                 // read the potentiometer input pin
Steering_Potentiometer = Steering_AD_Value*100/ADC_Bits;        // and convert it to 0-100%
/*
SpeedPID_Input = Speed_Measured;
SpeedPID_Setpoint = Driving_Speed_Request;
SteeringPID_Input = Steering_Potentiometer;
SteeringPID_Setpoint = Steering_Request;
SpeedPID.Compute();
SteeringPID.Compute();
Steering_Request = SteeringPID_Output;
Driving_Speed_Request = SpeedPID_Output;
*/

Steering_Difference = Steering_Request - Steering_Potentiometer;
Steering_Difference = abs(Steering_Difference);
ROS_Steering_Measured = (-ROS_Steering_Command_yIntercept+float(Steering_Potentiometer))/ROS_Steering_Command_Slope;   // Wheel steering angle in radians
    
#ifdef Simulation                                      // Compiler directives for simulation or not
  #define State1 Safety_SW_State == 0 || Safety_SW_State == 1  // Safety Switch OK or not
#else                                                  // Simulation
  #define State1 Safety_SW_State == 0                  // Safety Switch OK
#endif
   if (State1){
    if (Steering_Difference > Steering_Deadband){
     Steering();
    } else {
      Steering_Enable = 0;
    }
   Driving_Enable = 1;
  } else {
    Steering_Enable = 0;
    Driving_Enable = 0;
  }
  Driving();

}
