#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

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
    rgb[:, :, 0] = bgra[:, :, 2]  # Red
    rgb[:, :, 1] = bgra[:, :, 1]  # Green
    rgb[:, :, 2] = bgra[:, :, 0]  # Blue
    return rgb

class Renderer:
    def __init__(self):
        self.busy = False
        self.camera = None

class ThermalCameraPublisher(Node):
    def __init__(self):
        super().__init__('thermal_camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/seek/seek_node/image_thermal', 10)
        self.renderer = Renderer()

        self.manager = SeekCameraManager(SeekCameraIOType.USB)
        self.manager.register_event_callback(self.on_event, self.renderer)

        self.timer = self.create_timer(0.1, lambda: None)
        self.get_logger().info('Thermal camera publisher initialized.')

    def on_frame(self, camera, camera_frame, renderer):
        # Extract and process the frame
        frame = camera_frame.color_argb8888.data
        rgb_frame = bgra2rgb(frame)
        
        # Create Image message
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "thermal_camera"
        img_msg.height, img_msg.width, _ = rgb_frame.shape
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = 0
        img_msg.step = rgb_frame.shape[1] * 3
        img_msg.data = rgb_frame.tobytes()

        # Publish the Image message
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
