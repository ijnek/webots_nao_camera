import socket
import struct
from threading import Thread

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class WebotsNaoCamera(Node):

    def __init__(self, node_name='webots_nao_camera', **kwargs):
        super().__init__(node_name, **kwargs)

        # Create image publishers
        self._top_publisher = self.create_publisher(Image, 'image/top', 10)
        self._bottom_publisher = self.create_publisher(Image, 'image/bottom', 10)

        # Parameters
        top_address = self.declare_parameter('top_address', 'localhost').value
        bottom_address = self.declare_parameter('bottom_address', 'localhost').value
        top_port = self.declare_parameter('top_port', 10001).value
        bottom_port = self.declare_parameter('bottom_port', 10002).value

        self.get_logger().debug('top_port: {}'.format(top_port))
        self.get_logger().debug('bottom_port: {}'.format(bottom_port))

        # Connect to the camera server
        while(True):
            try:
                self._top_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._top_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._top_sock.connect((top_address, top_port))
                self._top_sock.settimeout(0.1)
                self._top_sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)

                self._bottom_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._bottom_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._bottom_sock.connect((bottom_address, bottom_port))
                self._bottom_sock.settimeout(0.1)
                self._bottom_sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)

                break
            except:
                print("Failed to connect to camera server. Retrying...")
                continue

        # Start threads to continuously poll
        self._top_thread = Thread(target=self._loop, args=(self._top_publisher, self._top_sock))
        self._top_thread.start()
        self._bottom_thread = Thread(target=self._loop, args=(self._bottom_publisher, self._bottom_sock))
        self._bottom_thread.start()

    def _loop(self, publisher, sock):

        while rclpy.ok():
            try:

                header = sock.recv(16)
                (magic, tick, camId, bpp, width, height) = struct.unpack("8sHBBHH", header)
                data = []
                while len(data) < bpp * width * height:
                    data += sock.recv(1024)

                # Convert data to ROS msg
                msg = Image()
                msg.height = height
                msg.width = width
                msg.encoding = "yuv422_yuy2"
                msg.step = width * bpp
                msg.data = data

                # Publish the image
                publisher.publish(msg)

            except TimeoutError:
                pass

def main(args=None):
    rclpy.init(args=args)
    webots_nao_camera = WebotsNaoCamera()
    rclpy.spin(webots_nao_camera)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
