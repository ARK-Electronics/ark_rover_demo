import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class MessageFilterPublisher(Node):

    def __init__(self):
        super().__init__('message_filter_publisher')

        self.bridge = CvBridge()

        self.create_subscription(CompressedImage, '/left/compressed', self.callback_left, 10)
        self.create_subscription(CompressedImage, '/color/compressed', self.callback_color, 10)
        self.create_subscription(CompressedImage, '/right/compressed', self.callback_right, 10)

        self.publisher_left = self.create_publisher(Image, '/left/uncompressed', 10)
        self.publisher_color = self.create_publisher(Image, '/color/uncompressed', 10)
        self.publisher_right = self.create_publisher(Image, '/right/uncompressed', 10)

    def decompress_and_publish(self, msg, publisher):
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert to ROS Image message
        image_message = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")

        # Publish decompressed image
        publisher.publish(image_message)
        self.get_logger().info("Running")

    def callback_left(self, msg):
        self.decompress_and_publish(msg, self.publisher_left)

    def callback_color(self, msg):
        self.decompress_and_publish(msg, self.publisher_color)

    def callback_right(self, msg):
        self.decompress_and_publish(msg, self.publisher_right)

def main(args=None):
    rclpy.init(args=args)
    node = MessageFilterPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
