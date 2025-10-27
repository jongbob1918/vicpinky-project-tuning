#!/usr/bin/env python3
"""
Leading 모드 테스트 클라이언트
목표 지점으로 로봇을 안내하도록 leading_server에 요청합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys


class TestLeadingClient(Node):
    """Leading 모드 테스트 클라이언트"""

    def __init__(self):
        super().__init__('test_leading_client')

        # Publisher for goal pose (leading_server will monitor and navigate)
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.get_logger().info('Test Leading Client initialized')

    def send_goal(self, x, y, yaw=0.0):
        """목표 지점 발행"""

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        import math
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal_pose)
        self.get_logger().info(f'Sent goal: x={x}, y={y}, yaw={yaw:.2f}')

        # Note: This publishes to /goal_pose, but leading_server uses NavigateToPose internally
        # For full integration, you need to call leading_server.navigate_to_goal() directly
        # or modify leading_server to subscribe to /goal_pose

        self.get_logger().warn('Note: You need to manually call leading_server.navigate_to_goal()')
        self.get_logger().warn('For now, use Nav2 GUI or command line to send NavigateToPose goal')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print('Usage: ros2 run vicpinky_navigation test_leading_client.py <x> <y> [yaw]')
        print('Example: ros2 run vicpinky_navigation test_leading_client.py 2.0 1.0 0.0')
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    node = TestLeadingClient()
    node.send_goal(x, y, yaw)

    # Keep node alive briefly to ensure message is sent
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
