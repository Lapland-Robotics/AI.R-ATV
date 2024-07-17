#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


class Esp32ControllerNode(Node):

    def __init__(self):
        super().__init__('esp32_controller_node')
        topic = "/atv/debug"
        self.get_logger().info('Esp32ControllerNode is listening to Topic -> ' + topic)
        self.sub = self.create_subscription(String, topic, self.chatter_callback, 5)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(String, topic, self.chatter_callback, qos_profile)

        # Create a publisher for the Twist message
        self.twist_pub = self.create_publisher(Twist, '/atv/ctrl_cmd', 10)
        
        # Create a timer to publish the Twist message every 15 seconds
        timer_period = 40.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.iteration = 0

    def chatter_callback(self, msg: String):
        self.get_logger().info(str(msg))

    def publish_twist(self, speed, angle):
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = angle
        self.twist_pub.publish(twist_msg)
        self.get_logger().info(f'Publishing Twist message with speed {speed} and angle {angle}')

    def timer_callback(self):
        self.publish_twist(0.0, 0.1)
        time.sleep(10)
        self.publish_twist(0.0, 0.0)
        time.sleep(10)
        self.publish_twist(0.0, -0.1)
        time.sleep(10)
        self.publish_twist(0.0, 0.0)
        time.sleep(10)


def main(args=None):
    rclpy.init(args=args)
    node = Esp32ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
