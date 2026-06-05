#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.image_pub = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        self.bridge = CvBridge()

        self.image_paths = [
            '/home/shiqintong/ROSPackageForImageVideo/image_publisher_pkg/image_publisher_pkg/food.jpg',
            '/home/shiqintong/ROSPackageForImageVideo/image_publisher_pkg/image_publisher_pkg/food2.jpg',
        ]

        self.image_index = 0
        self.timer = self.create_timer(1.0, self.publish_image)

    def publish_image(self):
        cv_image = cv2.imread(self.image_paths[self.image_index])

        if cv_image is None:
            self.get_logger().error(f"Failed to read image from path: {self.image_paths[self.image_index]}")
            return

        ros_image = self.bridge.cv2_to_compressed_imgmsg(cv_image, dst_format='jpeg')
        self.image_pub.publish(ros_image)
        self.get_logger().info(f"Compressed image published: {self.image_paths[self.image_index]}")

        self.image_index = (self.image_index + 1) % len(self.image_paths)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

