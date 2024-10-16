//Pins
#define CH1   18 //Throttle ch1
#define CH2   19 //Throttle ch2
#define DIG1  22 //Direction ch1
#define DIG2  23 //Direction ch2
#define AOUT1 25 //Output ch1
#define AOUT2 26 //Output ch2
 
//Constants
#define FREQ  490  //AnalogWrite frequency
#define MAX_T 2500 //Max signal threshold
#define MIN_T 500  //Min signal threshold
#define RESOLUTION 8 //PWM resolution (8-bit, range from 0-255)

float turnFactor = 0.5; 
//Variables
int xAxis, xRaw;
int yAxis, yRaw;
int motor1, motor2;

void setup() {
  Serial.begin(115200);
  Serial.println("Initialising...");
 
  //Initialising digital outputs
  pinMode(DIG1, OUTPUT);
  pinMode(DIG2, OUTPUT);
 
  //Initialising PWM on ESP32
  ledcSetup(0, FREQ, RESOLUTION); // Channel 0 for AOUT1
  ledcSetup(1, FREQ, RESOLUTION); // Channel 1 for AOUT2
  
  ledcAttachPin(AOUT1, 0); // Attach AOUT1 to channel 0
  ledcAttachPin(AOUT2, 1); // Attach AOUT2 to channel 1
 
  //initialising digital inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
}

void loop() {
  xRaw = pulseIn(CH1, HIGH);
  yRaw = pulseIn(CH2, HIGH);
 
  //X-axis control
  if(xRaw > MIN_T && xRaw < MAX_T && yRaw > MIN_T && yRaw < MAX_T)
  { 

    xAxis = map(xRaw,993,2016,-255,255);
    yAxis = map(yRaw,1027,2010,-255,255);
    
    getMotorValues(xAxis, yAxis, &motor1, &motor2);

    
    digitalWrite(DIG1, motor1 >= 0);
    ledcWrite(0, abs(motor1)); // Writing PWM to channel 0 (AOUT1)
    digitalWrite(DIG2, motor2 >= 0);
    ledcWrite(1, abs(motor2)); // Writing PWM to channel 1 (AOUT2)
 
    Serial.print("xAxis: ");
    Serial.print(xAxis);
    Serial.print(", yAxis: ");
    Serial.print(yAxis);
    Serial.print(", Motor1: ");
    Serial.print(motor1);
    Serial.print(", Motor2: ");
    Serial.println(motor2);
    delay(50);
  }
  
  else {
    if (xRaw < MIN_T || xRaw > MAX_T)
    {
      Serial.println("CHANNEL X DEAD");
      digitalWrite(DIG1, HIGH);
      ledcWrite(0, 0); // Stop PWM output on channel 0
    }
    else {
      Serial.println("AXIS CONTROL MAP ERROR");
    }

    if (yRaw < MIN_T || yRaw > MAX_T)
    {
    Serial.println("CHANNEL Y DEAD");
    digitalWrite(DIG2, HIGH);
    ledcWrite(1, 0); // Stop PWM output on channel 1
    }
    else {
      Serial.println("AXIS CONTROL MAP ERROR");
    }
  }
}

void getMotorValues(int x, int y, int *m1, int *m2) {
  
  int _x, _y;
  if(y + (abs(x) * turnFactor) > 255 || y - (abs(x) * turnFactor) < -255) {
    _x = x * (255 / (abs(y) + (abs(x) * turnFactor)));
    _y = y * (255 / (abs(y) + (abs(x) * turnFactor)));
    x=_x;
    y=_y;
  }
  
  *m1 = y + (x*turnFactor);
  *m2 = -y + (x*turnFactor);
}