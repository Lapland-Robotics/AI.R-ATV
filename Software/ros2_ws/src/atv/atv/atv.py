#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Esp32ControllerNode(Node):

    def __init__(self):
        super().__init__('atv')
        topic = "/debug"
        self.get_logger().info('Esp32ControllerNode is listening to Topic -> ' + topic)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(String, topic, self.chatter_callback, qos_profile)
        
        # Create a publisher for the Twist message
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a timer to publish the Twist message
        timer_period = 4.0  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        
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

    # demo program to move the steering
    def timer_callback(self):
        if self.iteration % 4 == 0:
            self.publish_twist(0.5, 0.5)
        if self.iteration % 4 == 1:
            self.publish_twist(0.5, -0.5)
        if self.iteration % 4 == 2:
            self.publish_twist(-0.5, 0.5)
        if self.iteration % 4 == 3:
            self.publish_twist(-0.5, -0.5)

        self.iteration += 1

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