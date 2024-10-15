//Pins
#define DIG1  22 //Direction ch1
#define DIG2  23 //Direction ch2
#define CH1   18 //Throttle ch1
#define CH2   19 //Throttle ch2
#define AOUT1 14 //Output ch1
#define AOUT2 15 //Output ch2
 
//Constants
#define FREQ  490  //AnalogWrite frequency
#define MAX_T 2500 //Max signal threshold
#define MIN_T 500  //Min signal threshold


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
 
  //Initialising analog outputs
  analogWriteFrequency(14, FREQ);
  analogWriteFrequency(15, FREQ);
 
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
    //Serial.print("X-Raw:");
    //Serial.println(xRaw);
    //Serial.print("Y-Raw:");
    //Serial.println(yRaw);
    xAxis = map(xRaw,993,2016,-255,255);
    yAxis = map(yRaw,1027,2010,-255,255);
    
    getMotorValues(xAxis, yAxis, &motor1, &motor2);

    
    digitalWrite(DIG1, motor1 >= 0);
    analogWrite(AOUT1, abs(motor1));
    digitalWrite(DIG2, motor2 >= 0);
    analogWrite(AOUT2, abs(motor2));
 
    Serial.print("xAxis:");
    Serial.println(xAxis);
    Serial.print("yAxis:");
    Serial.println(yAxis);
    Serial.print("Motor1:");
    Serial.println(motor1);
    Serial.print("Motor2:");
    Serial.println(motor2);
  }
  
  else {
    if (xRaw < MIN_T || xRaw > MAX_T)
    {
      Serial.println("CHANNEL X DEAD");
      digitalWrite(DIG1, HIGH);
      analogWrite(AOUT1, 0);
    }
    else {
      Serial.println("AXIS CONTROL MAP ERROR");
    }

    if (yRaw < MIN_T || yRaw > MAX_T)
    {
    Serial.println("CHANNEL Y DEAD");
    digitalWrite(DIG2, HIGH);
    analogWrite(AOUT2, 0);
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