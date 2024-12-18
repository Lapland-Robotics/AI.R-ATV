import os
import rclpy
import cv2
import threading
import time

from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from datetime import datetime
from concurrent.futures import Future
import Jetson.GPIO as GPIO
import numpy as np
from PIL import Image

class DataCollector(Node):

    def __init__(self):
        super().__init__('DataCollector')

        # GPIO setup
        self.BUTTON_PIN = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.BUTTON_PIN, GPIO.IN)
        
        self.zed2_left_rgb = None
        self.zed2_right_rgb = None
        self.seek_thermal = None
        
        self.save_dir = ""
        self.get_logger().info('Data Collector node is initialized.')

        self.input_thread = threading.Thread(target=self.input_handler)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_handler(self):
        while True:
            if self.snapshot_trigger():
                self.get_logger().info("Button Pressed! Taking snapshot soon...")
                time.sleep(10)
                self.zed2_left_rgb = self.retrieve_message_by_topic('/zed/zed_node/left_raw/image_raw_color', Image)
                self.zed2_right_rgb = self.retrieve_message_by_topic('/zed/zed_node/right_raw/image_raw_color', Image)
                self.seek_thermal = self.retrieve_message_by_topic('/seek/seek_node/image_thermal', ImageMsg)
                
                if self.zed2_left_rgb and self.zed2_right_rgb:
                    self.save_dir = self.gen_folder()
                    self.save_image(self.zed2_left_rgb, "left_rgb.png")
                    self.save_image(self.zed2_right_rgb, "right_rgb.png")
                    self.save_image(self.seek_thermal, "thermal.png") 
                else:
                    self.get_logger().info("Could not fetch images from both topics (possibly no messages published yet).")

    def snapshot_trigger(self):
        # If the button is pressed, GPIO input is HIGH
        if GPIO.input(self.BUTTON_PIN) == GPIO.HIGH:
            return True
        else:
            return False

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
        srv = os.path.join("/", "home/robotics")
        dataset_path = os.path.join(srv, "ATV/Dataset")
        os.makedirs(dataset_path, exist_ok=True)
        
        now = datetime.now()
        timestamp_folder = now.strftime("%f-%S-%M-%H_%d-%m-%Y")
        image_folder = os.path.join(dataset_path, timestamp_folder)
        os.makedirs(image_folder, exist_ok=True)
        
        self.get_logger().info(f'Created folder: {image_folder}')
        return image_folder

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
