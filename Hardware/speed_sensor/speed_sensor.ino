#include <ESP32TimerInterrupt.h>

#define HALL_PIN 32

// Volatile variables for interrupt use
volatile unsigned long toothCount = 0;
volatile unsigned long lastEdgeTime = 0;

// 2000 is 2ms per sample. This debounce threshold prevents false triggers.
const unsigned long BOUNCE_THRESHOLD_US = 2000;

// Create an instance of the ESP32 hardware timer (using timer 0)
ESP32Timer ITimer(0);

// Variable to store the previous state of the hall sensor
volatile int lastHallState = LOW;

// Timer ISR: polls the sensor pin to detect rising edges
void IRAM_ATTR onTimer() {
  int currentState = digitalRead(HALL_PIN);
  
  // Detect a rising edge: current HIGH and previous LOW
  if (currentState == HIGH && lastHallState == LOW) {
    unsigned long currentMicros = micros();
    // Apply debouncing: only count if enough time has passed since the last edge
    if ((currentMicros - lastEdgeTime) > BOUNCE_THRESHOLD_US) {
      toothCount++;
      lastEdgeTime = currentMicros;
    }
  }
  
  // Save the current state for the next timer check
  lastHallState = currentState;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(HALL_PIN, INPUT_PULLUP);
  // Initialize lastHallState from the sensor's current reading
  lastHallState = digitalRead(HALL_PIN);
  
  // Set up the timer interrupt to call onTimer() every 500 microseconds
  if (ITimer.attachInterruptInterval(500, onTimer)) {
    Serial.println("Timer interrupt started successfully");
  } else {
    Serial.println("Error: cannot set timer interrupt");
  }
  
  Serial.println("Starting gear tooth count with debounce using ESP32TimerInterrupt...");
}

void loop() {
  // Measure pulses every 100 ms (0.1 second)
  static unsigned long lastMillis = 0;
  static unsigned long lastToothCount = 0;

  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 100) {
    lastMillis = currentMillis;

    // Calculate the number of pulses in the last 0.1 seconds
    unsigned long pulsesInInterval = toothCount - lastToothCount;
    lastToothCount = toothCount;

    // Calculate pulses per minute (PPM)
    float speedPPM = pulsesInInterval * 600.0;

    // Print the running total and the speed (PPM)
    Serial.print("Teeth total: ");
    Serial.print(toothCount);
    Serial.print(" | Speed (PPM): ");
    Serial.println(speedPPM);
  }
}
