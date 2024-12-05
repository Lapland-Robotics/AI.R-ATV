import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose
import numpy as np
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('imu_to_odometry')
        
        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Subscribers and Publishers
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_sub = self.create_subscription(Imu, '/snower/ouster/imu', self.handle_imu, qos_profile)
        self.odom_pub = self.create_publisher(Odometry, '/snower/odom', 10)
        
        # Integration Variables
        self.prev_time = None
        self.position = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, vz]
        self.orientation = np.array([0.0, 0.0, 0.0])  # Roll, Pitch, Yaw
        
        self.get_logger().info("IMU to Odometry node started.")

    def handle_imu(self, msg: Imu):
        # Get the current timestamp
        current_time = self.get_clock().now()
        
        if self.prev_time is None:
            self.prev_time = current_time
            return

        # Compute time delta
        dt = (current_time - self.prev_time).nanoseconds * 1e-9  # Convert to seconds
        self.prev_time = current_time

        # Angular velocity (rad/s)
        angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

        # Linear acceleration (m/s^2)
        linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        
        # Update orientation using gyroscope data
        self.orientation += angular_velocity * dt
        q = quaternion_from_euler(0.0, 0.0, self.orientation[2])   # Flat surface

        # Update velocity and position using accelerometer data
        self.velocity += linear_acceleration * dt
        self.position += self.velocity * dt
        
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Set pose
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = 0.0  # Flat surface
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Set velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = 0.0  # Flat surface
        odom_msg.twist.twist.angular.x = 0.0  # Flat surface
        odom_msg.twist.twist.angular.y = 0.0  # Flat surface
        odom_msg.twist.twist.angular.z = angular_velocity[2]

        # Publish Odometry
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
