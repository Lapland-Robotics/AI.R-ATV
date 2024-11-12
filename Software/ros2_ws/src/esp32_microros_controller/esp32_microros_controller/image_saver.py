import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from datetime import datetime

class ImageSaver(Node):

    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left_raw/image_raw_color',
            self.listener_callback,
            10
        )
        self.br = CvBridge()
        self.image_counter = 0
        self.save_dir = ""
        self.get_logger().info('Data Collecting node is initialized and listening for sensors.')

    def listener_callback(self, msg):
        self.get_logger().info('Received an image message.')
        try:
            input("Press Enter to continue...")
            self.save_dir = self.gen_folder()
            self.save_rgb_left(msg)
            # self.save_rgb_right()
            # self.save_thermal()
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def gen_folder(self):
        desktop = os.path.join(os.path.expanduser("~"), "Desktop")
        dataset_path = os.path.join(desktop, "Dataset")
        os.makedirs(dataset_path, exist_ok=True)
        
        now = datetime.now()
        timestamp_folder = now.strftime("%f-%S-%M-%H_%d-%m-%Y")
        image_folder = os.path.join(dataset_path, timestamp_folder)
        os.makedirs(image_folder, exist_ok=True)
        
        self.get_logger().info(f'Created folder: {image_folder}')
        return image_folder

    def save_rgb_left(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        filename = os.path.join(self.save_dir, f'rgb_left.png')
        try:
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved image: {filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    try:
        rclpy.spin(image_saver)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        image_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



        

