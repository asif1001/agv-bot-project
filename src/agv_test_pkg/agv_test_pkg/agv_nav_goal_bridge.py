from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class AgvNavGoalBridge(Node):
    def __init__(self) -> None:
        super().__init__('agv_nav_goal_bridge')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.active_goal_handle = None

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.get_logger().info(
            'agv_nav_goal_bridge started. Use RViz 2D Goal Pose to send navigation goals.'
        )

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        self.send_goal(msg)

    def send_goal(self, pose: PoseStamped) -> None:
        if not self.action_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 action server...')
            self.action_client.wait_for_server()

        if self.active_goal_handle is not None:
            self.get_logger().info('Canceling previous navigation goal.')
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending Nav2 goal in frame {pose.header.frame_id}: '
            f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}'
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal was rejected.')
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info('Nav2 goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging around action failures
            self.get_logger().error(f'Nav2 goal result failed: {exc}')
            return

        self.active_goal_handle = None
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 goal succeeded.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 goal was aborted.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 goal was canceled.')
        else:
            self.get_logger().info(f'Nav2 goal finished with status {status}.')

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance_remaining: Optional[float] = getattr(feedback, 'distance_remaining', None)
        if distance_remaining is not None:
            self.get_logger().debug(f'Distance remaining: {distance_remaining:.2f} m')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgvNavGoalBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
