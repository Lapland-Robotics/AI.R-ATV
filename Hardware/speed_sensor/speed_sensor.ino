#include <Arduino.h>

#define HALL_PIN 33      // Sensor input pin
#define THRESHOLD 1.65   // Voltage threshold between LOW and HIGH (adjust if needed)

unsigned long pulseCount = 0; // Total pulse count
bool lastStateHigh = false;   // Tracks the previous sensor state

// Variables to compute pulses per second (PPS)
unsigned long lastMillis = 0;
unsigned long lastPulseCount = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // Set ADC resolution to 12 bits (0 - 4095)
}

void loop() {
  // Read the raw ADC value and convert it to voltage
  int rawValue = analogRead(HALL_PIN);
  float voltage = rawValue * (3.3 / 4095.0);

  // Determine if the current state is HIGH (voltage above threshold)
  bool currentStateHigh = (voltage >= THRESHOLD);

  // Detect a rising edge: transition from LOW to HIGH
  if (!lastStateHigh && currentStateHigh) {
    pulseCount++;
    //Serial.print("Pulse detected. Count: ");
    Serial.println(pulseCount);
  }
  // Update last state for next iteration
  lastStateHigh = currentStateHigh;

  // Every 1000 milliseconds, calculate and print pulses per second
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 1000) {
    unsigned long pulsesPerSecond = pulseCount - lastPulseCount;
    lastPulseCount = pulseCount;
    lastMillis = currentMillis;
    
    //Serial.print("Pulses per second: ");
    //Serial.println(pulsesPerSecond);
  }

  delay(2);  // Adjust delay as needed for your sampling rate
}
