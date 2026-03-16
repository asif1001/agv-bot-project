import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloAgvNode(Node):
    def __init__(self) -> None:
        super().__init__('hello_agv_node')

        # Create a publisher that sends String messages on /agv_status.
        self.publisher_ = self.create_publisher(String, '/agv_status', 10)

        # Create a timer that runs every 2 seconds.
        self.timer = self.create_timer(2.0, self.publish_status)

        self.get_logger().info('hello_agv_node has started and is publishing to /agv_status')

    def publish_status(self) -> None:
        # Build the message and publish it.
        message = String()
        message.data = 'Hello from AGV'
        self.publisher_.publish(message)

        self.get_logger().info(f'Published message: {message.data}')


def main(args=None) -> None:
    rclpy.init(args=args)

    node = HelloAgvNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
