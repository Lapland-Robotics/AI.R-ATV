
#include <stdio.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

std_msgs__msg__Bool msg;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
int lastState = HIGH;
int currentState;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define BUTTON_PIN 21 

void setup() {
  
  set_microros_wifi_transports("Sohan_A54", "12345678", "192.168.102.38", 8888); // microros over wifi
  // set_microros_transports(); // microros over serial
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/trigger"));

  msg.data = false;

  pinMode(BUTTON_PIN, INPUT_PULLUP);

}

void loop() {

  currentState = digitalRead(BUTTON_PIN);

  if(lastState == LOW && currentState == HIGH){
    msg.data = true;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    delay(1000);
  }

  lastState = currentState;
}
