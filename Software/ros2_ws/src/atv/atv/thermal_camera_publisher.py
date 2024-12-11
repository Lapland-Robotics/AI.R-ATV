#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
from threading import Condition
from seekcamera import (
    SeekCameraIOType,
    SeekCameraColorPalette,
    SeekCameraManager,
    SeekCameraManagerEvent,
    SeekCameraFrameFormat,
)

def bgra2rgb(bgra):
    # Convert BGRA (ARGB) to RGB
    rgb = np.zeros((bgra.shape[0], bgra.shape[1], 3), dtype="uint8")
    # ARGB8888 is in order: Blue, Green, Red, Alpha
    # We map it to RGB
    rgb[:, :, 0] = bgra[:, :, 2]  # Red
    rgb[:, :, 1] = bgra[:, :, 1]  # Green
    rgb[:, :, 2] = bgra[:, :, 0]  # Blue
    return rgb

class Renderer:
    def __init__(self):
        self.busy = False
        self.camera = None
        #self.frame_condition = Condition()

class ThermalCameraPublisher(Node):
    def __init__(self):
        super().__init__('thermal_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'thermal/image_raw', 10)
        self.br = CvBridge()
        self.renderer = Renderer()

        self.manager = SeekCameraManager(SeekCameraIOType.USB)
        self.manager.register_event_callback(self.on_event, self.renderer)

        # Use a timer to keep the node active
        self.timer = self.create_timer(0.1, lambda: None)

        self.get_logger().info('Thermal camera publisher initialized.')

    def on_frame(self, camera, camera_frame, renderer):
        frame = camera_frame.color_argb8888.data
        rgb_frame = bgra2rgb(frame)
        img_msg = self.br.cv2_to_imgmsg(rgb_frame, encoding='rgb8')
        self.publisher_.publish(img_msg)

    def on_event(self, camera, event_type, event_status, renderer):
        if event_type == SeekCameraManagerEvent.CONNECT and not renderer.busy:
            renderer.busy = True
            renderer.camera = camera
            camera.color_palette = SeekCameraColorPalette.TYRIAN
            camera.register_frame_available_callback(self.on_frame, renderer)
            camera.capture_session_start(SeekCameraFrameFormat.COLOR_ARGB8888)

        elif event_type == SeekCameraManagerEvent.DISCONNECT and renderer.camera == camera:
            camera.capture_session_stop()
            renderer.camera = None
            renderer.busy = False

        elif event_type == SeekCameraManagerEvent.ERROR:
            self.get_logger().error(f"Camera error: {event_status}")

def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
