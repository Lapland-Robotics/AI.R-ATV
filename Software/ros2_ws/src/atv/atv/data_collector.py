import os
import rclpy
import cv2
import threading
import time

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
from concurrent.futures import Future


class DataCollector(Node):

    def __init__(self):
        super().__init__('DataCollector')
        
        self.zed2_left_rgb = None
        self.zed2_right_rgb = None
        self.seek_thermal = None
        
        self.br = CvBridge()
        self.save_dir = ""
        self.get_logger().info('Data Collector node is initialized.')

        self.input_thread = threading.Thread(target=self.input_handler)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_handler(self):
        while True:
            if self.snapshot_trigger():
                time.sleep(10)
                self.zed2_left_rgb = self.retrieve_message_by_topic('/zed/zed_node/left_raw/image_raw_color', Image)
                self.zed2_right_rgb = self.retrieve_message_by_topic('/zed/zed_node/right_raw/image_raw_color', Image)
                
                if self.zed2_left_rgb and self.zed2_right_rgb:
                    self.save_dir = self.gen_folder()
                    self.save_rgb(self.zed2_left_rgb, "left_rgb.png")
                    self.save_rgb(self.zed2_right_rgb, "right_rgb.png")
                    # todo morrti save the thermal camara image using self.seek_thermal
                else:
                    self.get_logger().info("Could not fetch images from both topics (possibly no messages published yet).")

    def snapshot_trigger(self):
        # need to implement
        # temp impl as True for testing
        return True

    def retrieve_message_by_topic(self, topic_name, msg_type):
        future = Future()

        def once_callback(msg):
            future.set_result(msg)

        subscription = self.create_subscription(msg_type, topic_name, once_callback, 10)

        try:
            msg = future.result(timeout=2.0)
            return msg
        except Exception:
            self.get_logger().warn(f"Timeout while waiting for message on {topic_name}")
            return None
        finally:
            self.destroy_subscription(subscription)

    def gen_folder(self):
        app = os.path.join("/", "app")
        dataset_path = os.path.join(app, "Data/Dataset")
        os.makedirs(dataset_path, exist_ok=True)
        
        now = datetime.now()
        timestamp_folder = now.strftime("%f-%S-%M-%H_%d-%m-%Y")
        image_folder = os.path.join(dataset_path, timestamp_folder)
        os.makedirs(image_folder, exist_ok=True)
        
        self.get_logger().info(f'Created folder: {image_folder}')
        return image_folder

    def save_rgb(self, msg, fname):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        filename = os.path.join(self.save_dir, fname)
        try:
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved image: {filename}')
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
