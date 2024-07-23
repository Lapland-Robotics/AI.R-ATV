#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    // Serial.print("error \n");
    delay(100);
  }
}

void setup() {
  // Serial.begin(115200);
  // set_microros_wifi_transports("SSID", "password", "xxx.xxx.xxx.xxx", 8888); // microros over wifi
  set_microros_transports(); // microros over serial

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/atv/debug"));

  // Initialize the String message
  msg.data.data = (char *)malloc(50 * sizeof(char)); // Allocate memory for the string
  msg.data.size = 0;
  msg.data.capacity = 50;
  snprintf(msg.data.data, msg.data.capacity, "Message: %d", 0); // Initial message
}

void loop() {
    static int count = 0;
    snprintf(msg.data.data, msg.data.capacity, "Message: %d", count);
    msg.data.size = strlen(msg.data.data);
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    delay(100);
    count++;
}
