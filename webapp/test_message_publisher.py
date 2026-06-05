import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ServerCommPublisher(Node):
    def __init__(self):
        super().__init__('server_comm_publisher')
        self.publisher_ = self.create_publisher(String, '/ServerComm', 10)
        self.timer = self.create_timer(2.0, self.publish_message)
        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello from ROS2! Message number: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ServerCommPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

