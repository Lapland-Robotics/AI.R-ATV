#include <Arduino.h>

#define HALL_PIN 33      // Sensor input pin
#define THRESHOLD 1.65   // Voltage threshold between LOW and HIGH (adjust if needed)

unsigned long pulseCount = 0; // Count of pulses (gear teeth)
bool lastStateHigh = false;   // Track the previous sensor state

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // 12-bit resolution (0 - 4095)
}

void loop() {
  // Read the raw ADC value and convert to voltage
  int rawValue = analogRead(HALL_PIN);
  float voltage = rawValue * (3.3 / 4095.0);

  // Determine if the current state is HIGH (above threshold)
  bool currentStateHigh = (voltage >= THRESHOLD);

  // Detect a rising edge: transition from LOW to HIGH
  if (!lastStateHigh && currentStateHigh) {
    pulseCount++;
    Serial.print("Pulse detected. Count: ");
    Serial.println(pulseCount);
  }

  // Update the last state for the next iteration
  lastStateHigh = currentStateHigh;

  // Optional: Print the voltage (if you still want to graph it)
  // Serial.print("Voltage: ");
  // Serial.println(voltage);

  delay(2);  // Adjust the delay as needed for your sampling rate
}
