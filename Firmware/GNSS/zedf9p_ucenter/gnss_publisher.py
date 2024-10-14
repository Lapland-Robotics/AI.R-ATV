from ublox_gps import UbloxGps
import serial
import rclpy, rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatStatus

# Can also use SPI here - import spidev
# I2C is not supported

port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
gps = UbloxGps(port)

# ROS message attributes : NavSatStatus, latitude, longitude, altitude, position_covariance
keys = ["lat", "lon", "height", "pDOP"]

class GpsNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'snower/gps', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
      try: 
  
        coords = gps.geo_coords()

        lat = coords.lat
        lon = coords.lon
        height = float(coords.height / 1000)
        pDOP = coords.pDOP

        cov = gps.geo_cov()
        cov = cov[:3] + cov[4:10]
        cov = [float(entry) for entry in cov]

        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position in degrees.
        msg.latitude = lat
        msg.longitude = lon

        # Altitude in metres.
        msg.altitude = height

        # Position covariance (???)
        msg.position_covariance = cov
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        print(f"Latitude: {lat}, Longitude: {lon}, Height: {height}, PDOP: {pDOP}")
        self.publisher_.publish(msg)
        self.best_pos_a = None
      #except (ValueError, IOError) as err:
      except:
        pass


def main(args=None):
    rclpy.init(args=args)

    gps_node = GpsNode()

    rclpy.spin(gps_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
