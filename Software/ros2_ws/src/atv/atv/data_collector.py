import os
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image as ImageMsg, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from datetime import datetime
import numpy as np
from PIL import Image
import pcl

class DataCollector(Node):

    def __init__(self):
        super().__init__('DataCollector')

        topic = "/trigger"
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Bool, topic, self.trigger_callback, qos_profile)

        self.latest_messages = {}  # Store the latest messages from topics

        # Create persistent subscriptions
        self.create_subscription(ImageMsg, '/zed/zed_node/left/image_rect_color',
                                 lambda msg: self.store_latest_message('/zed/zed_node/left/image_rect_color', msg), qos_profile)
        self.create_subscription(ImageMsg, '/zed/zed_node/right/image_rect_color',
                                 lambda msg: self.store_latest_message('/zed/zed_node/right/image_rect_color', msg), qos_profile)
        self.create_subscription(ImageMsg, '/seek/seek_node/image_thermal',
                                 lambda msg: self.store_latest_message('/seek/seek_node/image_thermal', msg), qos_profile)
        self.create_subscription(PointCloud2, '/ouster/points',
                                 lambda msg: self.store_latest_message('/ouster/points', msg), qos_profile)
     
        self.save_dir = ""

    def store_latest_message(self, topic_name, msg):
        self.latest_messages[topic_name] = msg

    def trigger_callback(self, msg: Bool):
        self.get_logger().info("Button Pressed! Taking snapshot...")
        self.save_dir = self.gen_folder()

        # Use stored messages instead of waiting for new ones
        self.save_image(self.latest_messages.get('/zed/zed_node/left/image_rect_color'), "left_rgb.png")
        self.save_image(self.latest_messages.get('/zed/zed_node/right/image_rect_color'), "right_rgb.png")
        self.save_image(self.latest_messages.get('/seek/seek_node/image_thermal'), "thermal.png")
        self.save_point_cloud(self.latest_messages.get('/ouster/points'), "lidar.pcd")

    def gen_folder(self):
        srv = os.path.join("/", "home/robotics")
        dataset_path = os.path.join(srv, "ATV/Dataset")
        os.makedirs(dataset_path, exist_ok=True)
        
        now = datetime.now()
        timestamp_folder = now.strftime("%f-%S-%M-%H_%d-%m-%Y")
        image_folder = os.path.join(dataset_path, timestamp_folder)
        os.makedirs(image_folder, exist_ok=True)
        
        self.get_logger().info(f'Created folder: {image_folder}')
        return image_folder

    def save_point_cloud(self, msg: PointCloud2, fname: str):
        try:
            # Extract points from the PointCloud2 message
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

            if not points:
               self.get_logger().warn("No points found in the point cloud message.")
               return
        
            # Convert to a plain 2D numpy array of shape (N, 3) with dtype float32
            pc_array = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)

            # Create a PCL point cloud object
            pcl_cloud = pcl.PointCloud()
            pcl_cloud.from_array(pc_array)

            # Save the PCL point cloud to a .pcl file
            filename = os.path.join(self.save_dir, fname)
            pcl.save(pcl_cloud, filename)
            self.get_logger().info(f"Point cloud saved to {filename}")
        except Exception as e:
            self.get_logger().error(f'Error saving point cloud: {e}')     

    def save_image(self, msg, fname):
        try:
            # Extract image properties from the message
            width = msg.width
            height = msg.height
            encoding = msg.encoding
            step = msg.step

            # Check encoding and set the appropriate mode
            if encoding == "rgb8":
                mode = "RGB"
            elif encoding == "bgr8":
                mode = "BGR"
            elif encoding == "bgra8":
                mode = "BGRA"
            elif encoding == "mono8":
                mode = "L"
            else:
                raise ValueError(f"Unsupported image encoding: {encoding}")

            # Convert the raw data into a NumPy array
            img_data = np.frombuffer(msg.data, dtype=np.uint8)

            # Handle image shape based on encoding
            if mode in ["RGB", "BGR", "BGRA"]:
                img_data = img_data.reshape((height, width, -1))  # Auto-detect channels
            elif mode == "L":
                img_data = img_data.reshape((height, width))

            # Convert BGR to RGB if needed (PIL uses RGB format)
            if mode == "BGR":
                img_data = img_data[:, :, ::-1]
            elif mode == "BGRA":
                img_data = img_data[:, :, :3]  # Drop alpha channel
                img_data = img_data[:, :, ::-1]  # Convert to RGB
                mode = "RGB"

            # Create a PIL Image and save it
            image = Image.fromarray(img_data, mode)
            filename = os.path.join(self.save_dir, fname)
            image.save(filename)
            self.get_logger().info(f"Image saved to {fname}")

        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
