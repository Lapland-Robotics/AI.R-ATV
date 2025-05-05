import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MapToOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('map_to_odom_broadcaster')

        # Initialize the Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the /current_pose topic
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(PoseStamped, '/current_pose', self.handle_current_pose, qos_profile)

        # Store the latest transform
        self.latest_transform = TransformStamped()

    def handle_current_pose(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

        # Store the latest transform for debugging or further use
        self.latest_transform = t

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
