import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageResizerNode(Node):

    def __init__(self):
        super().__init__('image_resizer')
        self.declare_parameter('input_topic', '/depthai_examples/color/image')
        self.declare_parameter('output_topic', 'color/image_resized')
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.callback,
            10)

    def callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Resize the image to 480p
        resized_image = cv2.resize(cv_image, (320, 240))

        # Convert the resized OpenCV image back to a ROS Image message
        resized_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")

        # Publish the resized image
        self.publisher_.publish(resized_msg)


def main(args=None):
    rclpy.init(args=args)

    image_resizer_node = ImageResizerNode()

    rclpy.spin(image_resizer_node)

    # Destroy the node explicitly
    image_resizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
