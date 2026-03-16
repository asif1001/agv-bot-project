import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


INSTRUCTIONS = """
AGV Keyboard Teleop
-------------------
Use these keys:
  w : move forward
  s : move backward
  a : turn left
  d : turn right
  x : stop
  q : quit

This node publishes Twist messages on /cmd_vel.
"""


class AgvKeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('agv_keyboard_teleop')

        # Publish velocity commands that a mobile robot controller can use.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.5
        self.angular_speed = 1.0

        self.get_logger().info('agv_keyboard_teleop started. Publishing to /cmd_vel')

    def publish_command(self, linear_x: float, angular_z: float) -> None:
        # Create and send one Twist message based on the pressed key.
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_.publish(twist)

        self.get_logger().info(
            f'Published cmd_vel -> linear.x: {twist.linear.x:.2f}, angular.z: {twist.angular.z:.2f}'
        )


def get_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    readable, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if readable else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None) -> None:
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)

    node = AgvKeyboardTeleop()
    print(INSTRUCTIONS)

    try:
        while rclpy.ok():
            key = get_key(settings).lower()

            if key == 'w':
                node.publish_command(node.linear_speed, 0.0)
            elif key == 's':
                node.publish_command(-node.linear_speed, 0.0)
            elif key == 'a':
                node.publish_command(0.0, node.angular_speed)
            elif key == 'd':
                node.publish_command(0.0, -node.angular_speed)
            elif key == 'x':
                node.publish_command(0.0, 0.0)
            elif key == 'q':
                node.publish_command(0.0, 0.0)
                break
    except KeyboardInterrupt:
        node.publish_command(0.0, 0.0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
