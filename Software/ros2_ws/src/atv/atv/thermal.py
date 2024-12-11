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

# Seek Thermal imports
import numpy
from PIL import Image as PILImage
from seekcamera import (
    SeekCameraIOType,
    SeekCameraColorPalette,
    SeekCameraManager,
    SeekCameraManagerEvent,
    SeekCameraFrameFormat,
    SeekCameraShutterMode,
    SeekCamera,
    SeekFrame,
)

COLOR_PALETTES = {
    "w": SeekCameraColorPalette(0),  # White Hot
    "b": SeekCameraColorPalette(1),  # Black Hot
    "i": SeekCameraColorPalette(2),  # Iron
    "r": SeekCameraColorPalette(3),  # Rainbow
    "c": SeekCameraColorPalette(4),  # Cool
    "t": SeekCameraColorPalette(5),  # Tyrian
    "h": SeekCameraColorPalette(6),  # Hotwrite
    "g": SeekCameraColorPalette(7),  # Glow
    "d": SeekCameraColorPalette(8),  # Dynamic Range
}


def bgra2rgb(bgra):
    """Convert BGRA to RGB."""
    row, col, ch = bgra.shape
    assert ch == 4, "ARGB image should have 4 channels."
    rgb = numpy.zeros((row, col, 3), dtype="uint8")
    # BGRA is in the order: Blue, Green, Red, Alpha
    rgb[:, :, 0] = bgra[:, :, 2]  # Red
    rgb[:, :, 1] = bgra[:, :, 1]  # Green
    rgb[:, :, 2] = bgra[:, :, 0]  # Blue (swapped)
    return rgb


class Renderer:
    """Holds camera and image data for the thermal camera."""
    def __init__(self):
        self.busy = False
        self.frame = None
        self.camera = None
        self.frame_condition = threading.Condition()
        self.first_frame = True


class DataCollector(Node):

    def __init__(self):
        super().__init__('DataCollector')
        
        self.zed2_left_rgb = None
        self.zed2_right_rgb = None
        self.seek_thermal = None  # Will hold the latest thermal frame in RGB format (numpy array)
        self.br = CvBridge()
        self.save_dir = ""

        self.renderer = Renderer()
        self.palette_key = 't'  # Default to Tyrian
        self.get_logger().info('Data Collector node is initialized.')

        # Start thermal camera in a background thread
        self.thermal_thread = threading.Thread(target=self.init_thermal_camera, daemon=True)
        self.thermal_thread.start()

        self.input_thread = threading.Thread(target=self.input_handler)
        self.input_thread.daemon = True
        self.input_thread.start()

    def init_thermal_camera(self):
        # Initialize and run the Seek Thermal camera manager in the background
        with SeekCameraManager(SeekCameraIOType.USB) as manager:
            manager.register_event_callback(self.on_event, self.renderer)
            # Keep running indefinitely, waiting for events/frames
            # This loop will allow the callbacks to run asynchronously
            while rclpy.ok():
                # Wait a bit and let callbacks do their work
                time.sleep(0.1)
    
    def on_frame(self, _camera, camera_frame, renderer):
        """Async callback for when a new thermal frame is available."""
        with renderer.frame_condition:
            renderer.frame = camera_frame.color_argb8888
            renderer.frame_condition.notify()
            # Convert frame to RGB and store in self.seek_thermal
            self.seek_thermal = bgra2rgb(renderer.frame.data)

    def on_event(self, camera, event_type, event_status, renderer):
        """Async callback for camera events."""
        self.get_logger().info("{}: {}".format(str(event_type), camera.chipid))
        if event_type == SeekCameraManagerEvent.CONNECT:
            if renderer.busy:
                return
            renderer.busy = True
            renderer.camera = camera
            renderer.first_frame = True
            # Set default palette
            camera.color_palette = COLOR_PALETTES[self.palette_key]
            # Register frame callback
            camera.register_frame_available_callback(self.on_frame, renderer)
            # Start capturing frames
            camera.capture_session_start(SeekCameraFrameFormat.COLOR_ARGB8888)

        elif event_type == SeekCameraManagerEvent.DISCONNECT:
            if renderer.camera == camera:
                camera.capture_session_stop()
                renderer.camera = None
                renderer.frame = None
                renderer.busy = False

        elif event_type == SeekCameraManagerEvent.ERROR:
            self.get_logger().error("{}: {}".format(str(event_status), camera.chipid))

    def input_handler(self):
        while True:
            if self.snapshot_trigger():
                time.sleep(10)
                self.zed2_left_rgb = self.retrieve_message_by_topic('/zed/zed_node/left_raw/image_raw_color', Image)
                self.zed2_right_rgb = self.retrieve_message_by_topic('/zed/zed_node/right_raw/image_raw_color', Image)
                
                # Wait briefly for thermal frame if not yet received
                if self.zed2_left_rgb and self.zed2_right_rgb:
                    # Make sure thermal frame is available
                    if self.seek_thermal is not None:
                        self.save_dir = self.gen_folder()
                        self.save_rgb(self.zed2_left_rgb, "left_rgb.png")
                        self.save_rgb(self.zed2_right_rgb, "right_rgb.png")
                        self.save_thermal(self.seek_thermal, "thermal.png")
                    else:
                        self.get_logger().info("Thermal frame not yet available.")
                else:
                    self.get_logger().info("Could not fetch RGB images from both topics.")

    def snapshot_trigger(self):
        # Implement your own logic here (button press, service call, etc.)
        # Temporarily always True for demonstration
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
        desktop = os.path.join(os.path.expanduser("~"), "Desktop")
        dataset_path = os.path.join(desktop, "Dataset")
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

    def save_thermal(self, thermal_frame, fname):
        # thermal_frame is a numpy RGB array
        filename = os.path.join(self.save_dir, fname)
        try:
            cv2.imwrite(filename, thermal_frame)
            self.get_logger().info(f'Saved thermal image: {filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving thermal image: {e}')


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
