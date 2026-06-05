import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os
import time

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/video_stream',
            self.listener_callback,
            10
        )
        self.get_logger().info('VideoSubscriber node has started.')


        self.save_directory = os.path.expanduser('~/FeedingTestForWeb/video_stream_test/video_stream_test/')
        os.makedirs(self.save_directory, exist_ok=True)

    def listener_callback(self, msg):
        try:

            self.get_logger().info(f'Received message format: {msg.format}')
            self.get_logger().info(f'Received data length: {len(msg.data)} bytes')
            self.get_logger().info(f'First 20 bytes: {msg.data[:20]}')

            video_filename = os.path.join(self.save_directory, 'received_video.webm')
            with open(video_filename, 'wb') as video_file:
                video_file.write(bytearray(msg.data))

            self.get_logger().info(f'Saved video to: {video_filename}')


            cap = cv2.VideoCapture(video_filename)


            fps = cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f'Video FPS: {fps}')

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                # 显示视频帧
                cv2.imshow('Received Video Stream', frame)

                # 控制播放速度
                time.sleep(1 / fps)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f'Error processing video: {e}')

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)

    # 清理资源
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

