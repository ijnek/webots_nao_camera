import socket
import struct
import cv2
import numpy as np
import queue
from enum import Enum
from threading import Thread

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CamIndex(Enum):
    TOP = 0
    BOTTOM = 1



class ImageServer:
    def __init__(self, addr, img_w, img_h, camera):
        self.img_dim = (img_w, img_h)  # image dimensions
        self.img_bytes = img_w*img_h*2 # image size in bytes
        self.camera = camera           # camera
        self.running = True

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(addr)
        sock.listen()
        sock.settimeout(0.01)
        sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, (16+self.img_bytes)*4)
        self.sock = sock

        self.queue = queue.Queue(maxsize=3)
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()



    def send(self, tick, image):
        self.queue.put((tick, image))



    def stop(self):
        self.running = False
        self.thread.join()



    def bgra2yuv(self, buf):
        bgra = np.frombuffer(buf, dtype=np.uint8).reshape(*self.img_dim, 4)
        bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        y, u, v = cv2.split(cv2.cvtColor(bgra, cv2.COLOR_BGR2YUV))

        rows, cols = y.shape

        # Downsample u horizontally
        u = cv2.resize(u, (cols//2, rows), interpolation=cv2.INTER_LINEAR)

        # Downsample v horizontally
        v = cv2.resize(v, (cols//2, rows), interpolation=cv2.INTER_LINEAR)

        # Interleave u and v:
        uv = np.zeros_like(y)
        uv[:, 0::2] = u
        uv[:, 1::2] = v

        # Merge y and uv channels
        yuv422 = cv2.merge((y, uv))
        return yuv422



    def run(self):
        conn = None
        while self.running:
            try:
                tick, img = self.queue.get(timeout=0.1)
                self.queue.task_done()
            except queue.Empty:
                continue

            if conn:
                img = self.bgra2yuv(img)
                header = struct.pack("8sHBBHH", b'wbimage\x00', tick, self.camera.value, 2, *self.img_dim)

                try:
                    conn.send(header)
                    conn.send(img)
                except ConnectionError:
                    conn.close()
                    conn = None
            else:
                try:
                    (conn, addr) = self.sock.accept()
                except:
                    conn = None
                    continue

class WebotsNaoCamera(Node):

    def __init__(self, node_name='webots_nao_camera', **kwargs):
        super().__init__(node_name, **kwargs)

        # Create image publisher
        self._publisher = self.create_publisher(Image, 'image', 10)

        # Parameters
        port = self.declare_parameter('port', 10001).value
        self.get_logger().debug('port: {}'.format(port))

        addr = ("localhost", port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.connect(addr)
        self._sock.settimeout(0.1)
        self._sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        # sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, (16+img_bytes)*4)

        # Start thread to continuously poll
        self._loop_thread = Thread(target=self._loop)
        self._loop_thread.start()

    def _loop(self):

        while rclpy.ok():
            try:

                header = self._sock.recv(16)
                (magic, tick, camId, bpp, width, height) = struct.unpack("8sHBBHH", header)
                print('here', bpp * width * height)
                data = []
                while len(data) < bpp * width * height:
                    data += self._sock.recv(1024)
                self.get_logger().debug('received image')

                # Convert data to ROS msg
                msg = Image()
                msg.height = height
                msg.width = width
                msg.encoding = "yuv422_yuy2"
                msg.step = width * bpp
                msg.data = data
                # print(len(msg.data))

                # Publish the image
                self._publisher.publish(msg)
            except TimeoutError:
                pass

def main(args=None):
    rclpy.init(args=args)
    webots_nao_camera = WebotsNaoCamera()
    rclpy.spin(webots_nao_camera)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
