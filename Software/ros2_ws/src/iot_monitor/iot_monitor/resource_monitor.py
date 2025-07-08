#!/usr/bin/env python3
import os
import time
import logging

import psutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from jtop import jtop

class ResourceMonitorNode(Node):

    def __init__(self):
        super().__init__('resource_monitor')

        log_dir = os.path.realpath("src/iot_monitor/logs/")
        os.makedirs(log_dir, exist_ok=True)
        now = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.declare_parameter('test', 'test')
        test_name = self.get_parameter('test').get_parameter_value().string_value + '_'
        
        log_file = os.path.join(log_dir, test_name + now + '.log')
        self.file_logger = logging.getLogger('resource_monitor_file')
        self.file_logger.setLevel(logging.INFO)
        fh = logging.FileHandler(log_file, mode='a')
        fh.setFormatter(logging.Formatter('%(asctime)s  %(message)s', datefmt='%Y-%m-%d %H:%M:%S'))
        self.file_logger.addHandler(fh)

        net = psutil.net_io_counters()
        self.prev_sent = net.bytes_sent
        self.prev_recv = net.bytes_recv
        self.prev_time = time.time()
        self.jetson = jtop()  
        self.jetson.start()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        net = psutil.net_io_counters()
        now = time.time()
        dt = now - self.prev_time

        sent_diff = net.bytes_sent - self.prev_sent
        recv_diff = net.bytes_recv - self.prev_recv

        sent_mbps = (sent_diff * 8) / dt / 1e6
        recv_mbps = (recv_diff * 8) / dt / 1e6
        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent

        if self.jetson.ok:
            stats = self.jetson.stats
            gpu = stats.get('GPU', stats.get('gpu', 0.0))

        msg = (f'CPU: {cpu:.1f}%  |  GPU: {gpu:.1f}%  |  RAM: {mem:.1f}%  |  Network sent {sent_mbps:.2f} Mbps , Received {recv_mbps:.2f} Mbps')

        self.get_logger().info(msg)
        self.file_logger.info(msg)


        self.prev_time = now
        self.prev_sent = net.bytes_sent
        self.prev_recv = net.bytes_recv

    def destroy_node(self):
        self.jetson.stop()
        super().destroy_node()

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
