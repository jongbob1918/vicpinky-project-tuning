#!/usr/bin/env python3
"""
테스트용 사람 위치 발행 노드
실제 뎁스 카메라 대신 테스트용으로 사람 위치를 발행합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class TestPersonPublisher(Node):
    """테스트용 사람 위치를 발행하는 노드"""

    def __init__(self):
        super().__init__('test_person_publisher')

        # Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('person_distance', 1.0)  # meters behind robot
        self.declare_parameter('person_angle', 180.0)  # degrees (180 = behind)
        self.declare_parameter('base_frame', 'base_footprint')

        publish_rate = self.get_parameter('publish_rate').value
        self.person_distance = self.get_parameter('person_distance').value
        self.person_angle = self.get_parameter('person_angle').value
        base_frame = self.get_parameter('base_frame').value

        # Publisher
        self.person_pose_pub = self.create_publisher(
            PoseStamped,
            '/person_pose',
            10
        )

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_person_pose)

        self.get_logger().info('Test Person Publisher started')
        self.get_logger().info(f'  Publishing person at {self.person_distance}m, {self.person_angle}° from robot')
        self.get_logger().info('  Topic: /person_pose')
        self.get_logger().info('')
        self.get_logger().info('Commands to change person position:')
        self.get_logger().info('  ros2 param set /test_person_publisher person_distance 1.5')
        self.get_logger().info('  ros2 param set /test_person_publisher person_angle 90.0')

    def publish_person_pose(self):
        """사람 위치 발행"""

        # Get current parameters (allow dynamic changes)
        self.person_distance = self.get_parameter('person_distance').value
        self.person_angle = self.get_parameter('person_angle').value

        # Calculate person position in robot frame
        angle_rad = math.radians(self.person_angle)
        x = self.person_distance * math.cos(angle_rad)
        y = self.person_distance * math.sin(angle_rad)

        # Create pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_footprint'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0  # No rotation

        self.person_pose_pub.publish(pose_msg)

        # Log occasionally
        if self.get_clock().now().nanoseconds % 2_000_000_000 < 100_000_000:  # ~every 2 seconds
            self.get_logger().info(
                f'Publishing person pose: distance={self.person_distance:.2f}m, '
                f'angle={self.person_angle:.1f}°, position=({x:.2f}, {y:.2f})',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestPersonPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
