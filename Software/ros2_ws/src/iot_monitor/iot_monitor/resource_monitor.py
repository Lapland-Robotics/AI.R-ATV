#!/usr/bin/env python3
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import psutil

class ResourceMonitorNode(Node):

    def __init__(self):
        super().__init__('resource_monitor')

        net = psutil.net_io_counters()
        self.prev_sent = net.bytes_sent
        self.prev_recv = net.bytes_recv
        self.prev_time = time.time()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):


        net = psutil.net_io_counters()
        now = time.time()
        dt = now - self.prev_time

        sent_diff = net.bytes_sent - self.prev_sent
        recv_diff = net.bytes_recv - self.prev_recv

        sent_mbps = (sent_diff * 8) / dt / 1e6
        recv_mbps = (recv_diff * 8) / dt / 1e6
        cpu_usage = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        
        self.get_logger().info(f'Memory Usage: {mem}%')
        self.get_logger().info(f'CPU Usage: {cpu_usage}%')
        self.get_logger().info(f'Network ↑ {sent_mbps:.2f} Mbps  |  ↓ {recv_mbps:.2f} Mbps')

        self.prev_time = now
        self.prev_sent = net.bytes_sent
        self.prev_recv = net.bytes_recv


def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()