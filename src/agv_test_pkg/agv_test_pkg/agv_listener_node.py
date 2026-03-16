import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AgvListenerNode(Node):
    def __init__(self) -> None:
        super().__init__('agv_listener_node')

        # Subscribe to the /agv_status topic and call the callback for each message.
        self.subscription = self.create_subscription(
            String,
            '/agv_status',
            self.listener_callback,
            10,
        )

        self.get_logger().info('agv_listener_node is listening to /agv_status')

    def listener_callback(self, message: String) -> None:
        # Print each received message in the terminal.
        self.get_logger().info(f'Received message: {message.data}')


def main(args=None) -> None:
    rclpy.init(args=args)

    node = AgvListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()