import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomToBaseLinkBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_base_link_broadcaster')

        # Initialize the Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the /odom topic
        self.subscription = self.create_subscription(Odometry, '/snower/odom', self.handle_odom, 10)
    
    def handle_odom(self, msg: Odometry):
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseLinkBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
