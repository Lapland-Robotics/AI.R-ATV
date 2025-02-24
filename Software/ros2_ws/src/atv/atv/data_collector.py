import os
import rclpy
import cv2
import threading
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import Image as ImageMsg, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from datetime import datetime
from concurrent.futures import Future
import numpy as np
from PIL import Image
import pcl

class DataCollector(Node):

    def __init__(self):
        super().__init__('DataCollector')

        topic = "/trigger"
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Bool, topic, self.trigger_callback, qos_profile)

        self.zed2_left_rgb = None
        self.zed2_right_rgb = None
        self.seek_thermal = None
        self.ouster_lidar = None        
        self.save_dir = ""


    def trigger_callback(self, msg: Bool):
        self.get_logger().info("Button Pressed! Taking snapshot...")
        self.zed2_left_rgb = self.retrieve_message_by_topic('/zed/zed_node/left_raw/image_raw_color', Image)
        self.zed2_right_rgb = self.retrieve_message_by_topic('/zed/zed_node/right_raw/image_raw_color', Image)
        self.seek_thermal = self.retrieve_message_by_topic('/seek/seek_node/image_thermal', ImageMsg)
        self.ouster_lidar = self.retrieve_message_by_topic('/ouster/points', PointCloud2)
        
        self.save_dir = self.gen_folder()

        self.save_image(self.zed2_left_rgb, "left_rgb.png") if self.zed2_left_rgb else None
        self.save_image(self.zed2_right_rgb, "right_rgb.png") if self.zed2_right_rgb else None
        self.save_image(self.seek_thermal, "thermal.png") if self.seek_thermal else None
        self.save_point_cloud(self.ouster_lidar, "lidar.pcd") if self.ouster_lidar else None


    def retrieve_message_by_topic(self, topic_name, msg_type):
        future = Future()

        def once_callback(msg):
            future.set_result(msg)

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        subscription = self.create_subscription(msg_type, topic_name, once_callback,qos_profile)

        try:
            msg = future.result(timeout=2.0)
            return msg
        except Exception:
            self.get_logger().warn(f"Timeout while waiting for message on {topic_name}")
            return None
        finally:
            self.destroy_subscription(subscription)

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
            elif encoding == "mono8":
                mode = "L"
            else:
                raise ValueError(f"Unsupported image encoding: {encoding}")

            # Convert the raw data into a NumPy array
            img_data = np.frombuffer(msg.data, dtype=np.uint8)

            # Handle image shape based on encoding
            if mode in ["RGB", "BGR"]:
                img_data = img_data.reshape((height, width, 3))
            elif mode == "L":
                img_data = img_data.reshape((height, width))

            # Convert BGR to RGB if needed (PIL uses RGB format)
            if mode == "BGR":
                img_data = img_data[:, :, ::-1]

            # Create a PIL Image and save it
            image = Image.fromarray(img_data, mode="RGB" if mode in ["RGB", "BGR"] else mode)
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
