#include <NewPing.h>

#define SONAR_NUM 6      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define SAFTY_STOP 9      // Safety stop pin

const int SD = 60;       // Stopping distance for the robot in cm

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(2, 2, MAX_DISTANCE),
  NewPing(3, 3, MAX_DISTANCE),
  NewPing(4, 4, MAX_DISTANCE),
  NewPing(5, 5, MAX_DISTANCE),
  NewPing(6, 6, MAX_DISTANCE),
  NewPing(7, 7, MAX_DISTANCE),
};

void setup() {
  Serial.begin(9600);

  // Safety stop pin setup
  pinMode(SAFTY_STOP, OUTPUT);
  digitalWrite(SAFTY_STOP, LOW);
}

bool isObjectTooClose() {
  char _buffer[7];
  for (int i = 0; i < SONAR_NUM; i++) {
    int distance = sonar[i].ping_cm();
    Serial.print( sprintf(_buffer, "| %i : %i ", i, distance) ? _buffer : "" );
    if (0 < distance && distance < SD) {
      Serial.println("STOP");
      return true;
    }
    delay(50);
  }
  Serial.println();
  return false;
}

void loop() {
  // Check if any sensor detects an object closer than the stopping distance
  if (isObjectTooClose()) {
    digitalWrite(SAFTY_STOP, HIGH);  // Stop if any sensor detects an object within stopping distance
    // delay(1500);
  } else {
    digitalWrite(SAFTY_STOP, LOW);   // Keep moving if no object is detected within stopping distance
  }
}