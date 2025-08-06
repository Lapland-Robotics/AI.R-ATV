#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

class WheelOdomPublisher(Node):
    def __init__(self):
        super().__init__('wheel_odom_publisher')
        
        # Latest wheel speeds (m/s)
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.wheel_base = 0.68

        # Subscribers to the wheel speed topics
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT)
        self.create_subscription(Float32, '/speed/left', self.left_speed_callback, qos_profile)
        self.create_subscription(Float32, '/speed/right', self.right_speed_callback, qos_profile)

        # Publisher for the wheel odometry message
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        # Timer for publishing at 20 Hz (0.05 sec period)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def left_speed_callback(self, msg: Float32):
        self.left_speed = msg.data

    def right_speed_callback(self, msg: Float32):
        self.right_speed = msg.data

    def timer_callback(self):
        # Compute the linear velocity as the average of the left and right speeds.
        linear_velocity = (self.left_speed + self.right_speed) / 2.0
        angular_velocity = (self.right_speed - self.left_speed) / self.wheel_base

        # Prepare the Odometry message
        odom = Odometry()
        current_time = self.get_clock().now().to_msg()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Pose is set to a constant zero value as EKF handles pose integration.
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # Set twist: only the linear velocities are used.
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        # Angular velocity is set to zero (EKF uses the IMU for this)
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        # Publish the odometry message
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
