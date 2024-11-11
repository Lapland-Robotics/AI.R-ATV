#!/usr/bin/env python3
import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageSaver(Node):

    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left_gray/image_rect_gray',
            self.listener_callback,
            10
        )
        self.br = CvBridge()
        self.image_counter = 0
        self.save_dir = os.path.expanduser("~/Downloads")
        self.get_logger().info('ImageSaver node is initialized and listening for images.')

    def listener_callback(self, msg):
        self.get_logger().info('Received an image message.')
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self.get_logger().info('Converted ROS Image message to OpenCV image.')
            self.save_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def save_image(self, cv_image):
        filename = os.path.join(self.save_dir, f'image_{self.image_counter}.png')
        try:
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved image: {filename}')
            self.image_counter += 1
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
