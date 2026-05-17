import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PiMessageListener(Node):
    def __init__(self) -> None:
        super().__init__('pi_message_listener')
        self.subscription = self.create_subscription(
            String,
            '/pi_hello',
            self.listener_callback,
            10,
        )
        self.get_logger().info('pi_message_listener is listening to /pi_hello')

    def listener_callback(self, message: String) -> None:
        self.get_logger().info(f'Received from Pi: {message.data}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiMessageListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
