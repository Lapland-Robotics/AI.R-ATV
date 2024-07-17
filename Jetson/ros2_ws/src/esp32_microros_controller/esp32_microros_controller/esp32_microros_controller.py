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
        # topic = "/atv/debug"
        # self.get_logger().info('Esp32ControllerNode is listening to Topic -> ' + topic)
        # qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # self.sub = self.create_subscription(String, topic, self.chatter_callback, qos_profile)
        
        # Create a publisher for the Twist message
        self.twist_pub = self.create_publisher(Twist, '/atv/ctrl_cmd', 10)

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

def main(args=None):
    rclpy.init(args=args)
    node = Esp32ControllerNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            node.publish_twist(0.0, 0.03)
            time.sleep(4)
            node.publish_twist(0.0, 0.0)
            time.sleep(4)
            node.publish_twist(0.0, -0.03)
            time.sleep(4)
            node.publish_twist(0.0, 0.0)
            time.sleep(4)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
