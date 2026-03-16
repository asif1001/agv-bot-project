import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AgvGoalNavigator(Node):
    def __init__(self) -> None:
        super().__init__('agv_goal_nav')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_frame: Optional[str] = None

        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.scan: Optional[LaserScan] = None

        self.goal_tolerance = 0.20
        self.max_linear_speed = 0.35
        self.max_angular_speed = 1.1
        self.heading_gain = 1.4
        self.slow_turn_gain = 0.35
        self.avoid_distance = 0.75
        self.emergency_distance = 0.38

        self.get_logger().info(
            'agv_goal_nav started. Use RViz "2D Goal Pose" or "Publish Point" to send a goal.'
        )

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        self.set_goal(msg.pose.position.x, msg.pose.position.y, msg.header.frame_id or 'odom')

    def clicked_point_callback(self, msg: PointStamped) -> None:
        self.set_goal(msg.point.x, msg.point.y, msg.header.frame_id or 'odom')

    def set_goal(self, x: float, y: float, frame_id: str) -> None:
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().warn('Goal ignored because odometry is not ready yet.')
            return

        if frame_id in ('base_link', 'base_footprint'):
            world_x = self.x + x * math.cos(self.yaw) - y * math.sin(self.yaw)
            world_y = self.y + x * math.sin(self.yaw) + y * math.cos(self.yaw)
            self.goal_x = world_x
            self.goal_y = world_y
            self.goal_frame = 'odom'
        else:
            self.goal_x = x
            self.goal_y = y
            self.goal_frame = frame_id

        self.get_logger().info(
            f'New goal accepted at x={self.goal_x:.2f}, y={self.goal_y:.2f} in {self.goal_frame}.'
        )

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def clear_goal(self) -> None:
        self.goal_x = None
        self.goal_y = None
        self.goal_frame = None

    def min_range_in_window(self, start_angle: float, end_angle: float) -> float:
        if self.scan is None:
            return math.inf

        min_distance = math.inf
        angle = self.scan.angle_min

        for reading in self.scan.ranges:
            if math.isfinite(reading) and self.scan.range_min <= reading <= self.scan.range_max:
                wrapped = normalize_angle(angle)
                if start_angle <= wrapped <= end_angle:
                    min_distance = min(min_distance, reading)
            angle += self.scan.angle_increment

        return min_distance

    def obstacle_avoidance_turn(self) -> float:
        front_min = self.min_range_in_window(-0.35, 0.35)
        if front_min >= self.avoid_distance:
            return 0.0

        left_min = self.min_range_in_window(0.35, 1.57)
        right_min = self.min_range_in_window(-1.57, -0.35)

        # Turn toward the side with more free space.
        if left_min > right_min:
            return 0.9
        return -0.9

    def control_loop(self) -> None:
        if None in (self.x, self.y, self.yaw):
            return

        if self.goal_x is None or self.goal_y is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Goal reached.')
            self.clear_goal()
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - self.yaw)

        front_min = self.min_range_in_window(-0.35, 0.35)
        avoidance_turn = self.obstacle_avoidance_turn()

        if front_min < self.emergency_distance:
            self.publish_cmd(0.0, avoidance_turn)
            return

        angular_cmd = self.heading_gain * heading_error + avoidance_turn
        angular_cmd = clamp(angular_cmd, -self.max_angular_speed, self.max_angular_speed)

        heading_scale = max(0.0, 1.0 - self.slow_turn_gain * abs(heading_error))
        obstacle_scale = 1.0
        if math.isfinite(front_min):
            obstacle_scale = clamp((front_min - self.emergency_distance) / 0.8, 0.0, 1.0)

        linear_cmd = min(self.max_linear_speed, 0.18 + 0.22 * distance)
        linear_cmd *= heading_scale * obstacle_scale

        if abs(heading_error) > 1.0 and front_min < self.avoid_distance:
            linear_cmd = 0.0

        self.publish_cmd(linear_cmd, angular_cmd)

    def destroy_node(self) -> bool:
        self.publish_cmd(0.0, 0.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgvGoalNavigator()

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
