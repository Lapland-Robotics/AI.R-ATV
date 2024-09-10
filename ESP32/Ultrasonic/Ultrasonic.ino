#include <Wire.h>        // Include Arduino Wire library
#include "Ultrasonic.h"  // Include Seeed Studio ultrasonic ranger library

#define NUM_ULTRASONICS 6 // number of ultrasonic sonsors
#define SAFTY_STOP 9      // Safety stop pin

const int RANGER_PINS[NUM_ULTRASONICS] = {2, 3, 4, 5, 6, 7}; // ultrasonic sonsors pins
const int SD = 50;       // Stopping distance for the robot in cm

// Create ultrasonic sensor objects
Ultrasonic ultrasonics[NUM_ULTRASONICS] = {
  Ultrasonic(RANGER_PINS[0]),
  Ultrasonic(RANGER_PINS[1]),
  Ultrasonic(RANGER_PINS[2]),
  Ultrasonic(RANGER_PINS[3]),
  Ultrasonic(RANGER_PINS[4]),
  Ultrasonic(RANGER_PINS[5])
};

void setup() {
  Serial.begin(115200);
  // Safety stop pin setup
  pinMode(SAFTY_STOP, OUTPUT);
  digitalWrite(SAFTY_STOP, LOW);
}

// Function to measure and check distances
bool isObjectTooClose() {
  char _buffer[7];
  for (int i = 0; i < NUM_ULTRASONICS; i++) {
    int distance = ultrasonics[i].MeasureInCentimeters();
    Serial.print( sprintf(_buffer, "| %i : %i ", i, distance) ? _buffer : "" );
    if (0 < distance && distance < SD) {
      return true;
    }
  }
  Serial.println();
  return false;
}

void loop() {
  // Check if any sensor detects an object closer than the stopping distance
  if (isObjectTooClose()) {
    digitalWrite(SAFTY_STOP, HIGH);  // Stop if any sensor detects an object within stopping distance
  } else {
    digitalWrite(SAFTY_STOP, LOW);   // Keep moving if no object is detected within stopping distance
  }

  delay(100);  // Add delay for stability
}
