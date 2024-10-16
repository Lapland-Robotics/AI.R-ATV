//Pins
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

float turnFactor = 0.5; 
//Variables
int angle, speed;
int motor1, motor2;

void setup() {
  Serial.begin(115200);
  Serial.println("Initialising...");
 
  //pin initialising
  pinMode(CH1RCPin, INPUT);
  pinMode(CH2RCPin, INPUT);
  pinMode(Motor1DirPin, OUTPUT);
  pinMode(Motor2DirPin, OUTPUT);
 
  //Initialising PWM on ESP32
  ledcSetup(0, FREQ, RESOLUTION); // Channel 0 for MotorSpeedPWM1
  ledcSetup(1, FREQ, RESOLUTION); // Channel 1 for MotorSpeedPWM2
  ledcAttachPin(Motor1SpeedPWMPin, 0); // Attach MotorSpeedPWM1 to channel 0
  ledcAttachPin(Motor2SpeedPWMPin, 1); // Attach MotorSpeedPWM2 to channel 1

}

void getRC(){
  int xRaw = pulseIn(CH1RCPin, HIGH);
  int yRaw = pulseIn(CH2RCPin, HIGH);
 
  //X-axis control
  if(xRaw > MIN_T && xRaw < MAX_T && yRaw > MIN_T && yRaw < MAX_T)
  {
    angle = map(xRaw,993,2016,-255,255);
    speed = map(yRaw,1027,2010,-255,255);
  }
  
  else {
    angle = 0;
    speed = 0;
    digitalWrite(Motor2DirPin, HIGH);
    ledcWrite(1, 0); // Stop PWM output on channel 1
    digitalWrite(Motor2DirPin, HIGH);
    ledcWrite(1, 0); // Stop PWM output on channel 1
  }
}

void driving() {
  
  int _x, _y;
  if(speed + (abs(angle) * turnFactor) > 255 || speed - (abs(angle) * turnFactor) < -255) {
    _x = angle * (255 / (abs(speed) + (abs(angle) * turnFactor)));
    _y = speed * (255 / (abs(speed) + (abs(angle) * turnFactor)));
    angle=_x;
    speed=_y;
  }
  
  motor1 = speed + (angle*turnFactor);
  motor2 = -speed + (angle*turnFactor);

  digitalWrite(Motor1DirPin, motor1 >= 0);
  ledcWrite(0, abs(motor1)); // Writing PWM to channel 0 (AOUT1)
  digitalWrite(Motor2DirPin, motor2 >= 0);
  ledcWrite(1, abs(motor2)); // Writing PWM to channel 1 (MotorSpeedPWM2)
}

void loop() {
  getRC();
  driving();
  delay(50);
}