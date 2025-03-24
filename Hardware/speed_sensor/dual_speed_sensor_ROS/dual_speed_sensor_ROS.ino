#include <Arduino.h>
#include <micro_ros_arduino.h> // micro-ROS library
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// --------------------------------------
// Hall Sensor Configuration
// --------------------------------------
#define HALL_PIN_1    33        // "Left" sensor
#define HALL_PIN_2    32        // "Right" sensor
#define THRESHOLD     1.65      // Voltage threshold for pulse detection

// ADC resolution on ESP32: 0..4095 => 0..3.3V
static const float ADC_MAX = 4095.0;
static const float VOLTAGE_REF = 3.3;

// --------------------------------------
// Pulse Count Tracking
// --------------------------------------
volatile unsigned long pulseCount1 = 0;  // Left sensor pulses
volatile unsigned long pulseCount2 = 0;  // Right sensor pulses

bool lastStateHigh1 = false;  // Previous sensor state for pin 33
bool lastStateHigh2 = false;  // Previous sensor state for pin 32

// For once-per-second reporting
unsigned long lastMillis = 0;
unsigned long lastCount1 = 0;
unsigned long lastCount2 = 0;

// --------------------------------------
// micro-ROS variables
// --------------------------------------
rcl_publisher_t speedLeftPublisher;
rcl_publisher_t speedRightPublisher;

std_msgs__msg__Float32 speedLeftMsg;
std_msgs__msg__Float32 speedRightMsg;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// --------------------------------------
// micro-ROS Helper Macros + Error Loop
// --------------------------------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// If something goes wrong, loop or reset
void errorLoop() {
  Serial.println("Error in micro-ROS. Halting or resetting...");
  delay(2000);
  ESP.restart();
}

// --------------------------------------
// Setup micro-ROS to run over Serial
// --------------------------------------
void setupMicroROS() {
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node_serial", "", &support));
  
  RCCHECK(
    rclc_publisher_init_best_effort(
      &speedLeftPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/snower/speed/left"
    )
  );

  RCCHECK(
    rclc_publisher_init_best_effort(
      &speedRightPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/snower/speed/right"
    )
  );

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait a moment for serial to come up

  // Set resolution for analog reading on the ESP32 (0..4095 range)
  analogReadResolution(12);

  // Initialize micro-ROS (wired)
  setupMicroROS();
  Serial.println("micro-ROS serial setup complete.");
}

void loop() {
  // ------------------------------------------------------------------------------
  // 1) Read sensor A (pin 33) using analogRead
  // ------------------------------------------------------------------------------
  int rawValue1 = analogRead(HALL_PIN_1);
  float voltage1 = rawValue1 * (VOLTAGE_REF / ADC_MAX);

  bool currentStateHigh1 = (voltage1 >= THRESHOLD);
  if (!lastStateHigh1 && currentStateHigh1) {
    pulseCount1++;
  }
  lastStateHigh1 = currentStateHigh1;

  // ------------------------------------------------------------------------------
  // 2) Read sensor B (pin 32) using analogRead
  // ------------------------------------------------------------------------------
  int rawValue2 = analogRead(HALL_PIN_2);
  float voltage2 = rawValue2 * (VOLTAGE_REF / ADC_MAX);

  bool currentStateHigh2 = (voltage2 >= THRESHOLD);
  if (!lastStateHigh2 && currentStateHigh2) {
    pulseCount2++;
  }
  lastStateHigh2 = currentStateHigh2;

  // ------------------------------------------------------------------------------
  // 3) Every 1000 ms, calculate pulses for the last second
  // ------------------------------------------------------------------------------
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 1000) {
    unsigned long pulsesLastSecond1 = pulseCount1 - lastCount1;
    unsigned long pulsesLastSecond2 = pulseCount2 - lastCount2;

    lastCount1 = pulseCount1;
    lastCount2 = pulseCount2;
    lastMillis = currentMillis;

    // Print to Serial Monitor (for debugging)
    Serial.print("Sensor A pulses in last second: ");
    Serial.println(pulsesLastSecond1);
    Serial.print("Sensor B pulses in last second: ");
    Serial.println(pulsesLastSecond2);
    Serial.println("--------");

    // ----------------------------------------------------------------------------
    // 4) Publish pulsesLastSecond via micro-ROS
    // ----------------------------------------------------------------------------
    speedLeftMsg.data = (float)pulsesLastSecond1;
    speedRightMsg.data = (float)pulsesLastSecond2;

    RCSOFTCHECK(rcl_publish(&speedLeftPublisher, &speedLeftMsg, NULL));
    RCSOFTCHECK(rcl_publish(&speedRightPublisher, &speedRightMsg, NULL));
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
// If the pulses aren't counting correctly, update this to be a bit smaller (just added it for debouncing)
  delay(2);
}
