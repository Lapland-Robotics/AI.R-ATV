#define HALL_PIN 32

// Volatile variables for interrupt use
volatile unsigned long toothCount = 0;
volatile unsigned long lastEdgeTime = 0;   

// 2000 is 2ms per sample, Might need to make it smaller if possible but this is for debounce.
// If debounce is not used, the sensor goes crazy
const unsigned long BOUNCE_THRESHOLD_US = 2000;  

void IRAM_ATTR onRisingEdge() {
  unsigned long currentMicros = micros();

  // Debounce checker
  if ((currentMicros - lastEdgeTime) > BOUNCE_THRESHOLD_US) {
    toothCount++;
    lastEdgeTime = currentMicros;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(HALL_PIN, INPUT_PULLUP);

  // Interrupt on RISING edge to increment toothCount
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onRisingEdge, RISING);

  Serial.println("Starting gear tooth count with debounce...");
}

void loop() {
  // measure pulses every 100 ms (0.1 second)
  static unsigned long lastMillis = 0;
  static unsigned long lastToothCount = 0;

  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 100) {
    lastMillis = currentMillis;

    // Number of pulses in the last 0.1 seconds
    unsigned long pulsesInInterval = toothCount - lastToothCount;
    lastToothCount = toothCount;

    // Calculate pulses per minute (PPM).
    float speedPPM = pulsesInInterval * 600.0;

    // Print running total and speed
    Serial.print("Teeth total: ");
    Serial.print(toothCount);
    Serial.print(" | Speed (PPM): ");
    Serial.println(speedPPM);

    Serial.println(speedPPM);
  }
}
