# Author: Andrey Li (12250249)
"""
video_source: plays a recorded video as a camera for the S1 camera path.

Publishes each frame of `video_path` as sensor_msgs/Image (bgr8, stamped on publication) on
`image_topic` at the file's frame rate, looping at the end. A separate topic from the
simulator's /camera/image_raw, so both can run at once.
"""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from smart_hub.qos import SENSOR


class VideoSource(Node):

    def __init__(self):
        super().__init__('video_source')
        self.declare_parameter('video_path', '')
        self.declare_parameter('image_topic', '/camera/video')
        self.declare_parameter('loop', True)
        self.cap = cv2.VideoCapture(self.get_parameter('video_path').value)
        if not self.cap.isOpened():
            raise ValueError('video_source: cannot open video_path')
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 10.0
        self.pub = self.create_publisher(Image, self.get_parameter('image_topic').value, SENSOR)
        self.create_timer(1.0 / fps, self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok and self.get_parameter('loop').value:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VideoSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
