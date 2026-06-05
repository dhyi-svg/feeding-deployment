#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import time

class VideoPublisher(Node):
    def __init__(self, video_path):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        self.timer = self.create_timer(1/30.0, self.publish_frame)  # 30 FPS
        self.cap = cv2.VideoCapture(video_path)
        self.get_logger().info("Video publisher node started")

    def publish_frame(self):
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open video file!")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video ended, restarting...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # Encode the frame as JPEG
        success, buffer = cv2.imencode('.jpg', frame)
        if not success:
            self.get_logger().error("Failed to encode frame")
            return

        # Create a CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info("Published video frame")

    def stop(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    video_path = "/home/shiqintong/ROStestFor/video_publisher_pkg/video_publisher_pkg/test.mp4"
    video_publisher = VideoPublisher(video_path)

    try:
        rclpy.spin(video_publisher)
    except KeyboardInterrupt:
        video_publisher.get_logger().info("Shutting down video publisher node")
        video_publisher.stop()
    finally:
        video_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

