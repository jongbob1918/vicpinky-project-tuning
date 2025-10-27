#!/usr/bin/env python3
"""
Leading Server - Robot leads a person to a destination
로봇이 목표지점으로 안내하면서 사람과 거리를 유지하고, 이탈 시 정지하는 노드

Author: Claude & User
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import math
import time


class LeadingServer(Node):
    """
    로봇이 사람을 안내(leading)하는 서버
    - 목표 지점으로 로봇이 이동
    - 사람과 일정 거리 유지 (desired_distance)
    - 사람이 이탈하면 정지 (max_person_distance)
    - 사람이 복귀하면 재개
    """

    def __init__(self):
        super().__init__('leading_server')

        # Parameters
        self.declare_parameter('controller_frequency', 10.0)
        self.declare_parameter('desired_distance', 1.0)  # 유지할 목표 거리 (m)
        self.declare_parameter('min_person_distance', 0.5)  # 최소 거리 (너무 가까우면 경고)
        self.declare_parameter('max_person_distance', 2.0)  # 최대 거리 (이탈 판정)
        self.declare_parameter('detection_timeout', 3.0)  # 사람 감지 타임아웃 (초)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('person_pose_topic', '/person_pose')

        self.controller_frequency = self.get_parameter('controller_frequency').value
        self.desired_distance = self.get_parameter('desired_distance').value
        self.min_person_distance = self.get_parameter('min_person_distance').value
        self.max_person_distance = self.get_parameter('max_person_distance').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.base_frame = self.get_parameter('base_frame').value
        self.person_pose_topic = self.get_parameter('person_pose_topic').value

        # State variables
        self.person_pose = None
        self.last_person_detection_time = None
        self.is_person_detected = False
        self.is_navigating = False
        self.nav_goal_handle = None

        # Subscribe to person pose (from depth camera)
        self.person_pose_sub = self.create_subscription(
            PoseStamped,
            self.person_pose_topic,
            self.person_pose_callback,
            10
        )

        # Publisher for cmd_vel (emergency stop if needed)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # NavigateToPose action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer for monitoring person distance
        self.monitor_timer = self.create_timer(
            1.0 / self.controller_frequency,
            self.monitor_person_distance
        )

        self.get_logger().info('Leading Server initialized')
        self.get_logger().info(f'  Desired distance: {self.desired_distance}m')
        self.get_logger().info(f'  Max distance (stop): {self.max_person_distance}m')
        self.get_logger().info(f'  Min distance (warning): {self.min_person_distance}m')
        self.get_logger().info(f'  Detection timeout: {self.detection_timeout}s')

    def person_pose_callback(self, msg: PoseStamped):
        """사람 위치 수신 콜백"""
        self.person_pose = msg
        self.last_person_detection_time = self.get_clock().now()
        self.is_person_detected = True

        # Calculate distance from robot (assuming pose is in base_frame or odom)
        distance = self.calculate_person_distance()

        # Log occasionally
        if self.get_clock().now().nanoseconds % 2_000_000_000 < 100_000_000:  # ~every 2 seconds
            self.get_logger().info(f'Person detected at distance: {distance:.2f}m', throttle_duration_sec=2.0)

    def calculate_person_distance(self):
        """로봇과 사람 사이의 거리 계산"""
        if self.person_pose is None:
            return float('inf')

        # Assuming person_pose is in base_footprint or we need to transform it
        # For simplicity, using direct distance calculation
        dx = self.person_pose.pose.position.x
        dy = self.person_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        return distance

    def is_person_detection_valid(self):
        """사람 감지가 유효한지 확인 (타임아웃 체크)"""
        if self.last_person_detection_time is None:
            return False

        time_since_detection = (self.get_clock().now() - self.last_person_detection_time).nanoseconds / 1e9
        return time_since_detection < self.detection_timeout

    def monitor_person_distance(self):
        """사람과의 거리를 모니터링하고 내비게이션 제어"""

        # Check if we're currently navigating
        if not self.is_navigating:
            return

        # Check person detection validity
        if not self.is_person_detection_valid():
            self.get_logger().warn(
                f'Person detection lost! (timeout: {self.detection_timeout}s) - STOPPING NAVIGATION',
                throttle_duration_sec=1.0
            )
            self.stop_navigation()
            self.is_person_detected = False
            return

        # Calculate current distance
        distance = self.calculate_person_distance()

        # Check if person is too far (lost)
        if distance > self.max_person_distance:
            self.get_logger().warn(
                f'Person too far away! ({distance:.2f}m > {self.max_person_distance}m) - STOPPING',
                throttle_duration_sec=0.5
            )
            self.pause_navigation()

        # Check if person is too close
        elif distance < self.min_person_distance:
            self.get_logger().info(
                f'Person too close! ({distance:.2f}m < {self.min_person_distance}m) - Slowing down',
                throttle_duration_sec=1.0
            )
            # TODO: Could send a slower speed command here

        # Person is in good range
        else:
            self.get_logger().debug(f'Person distance OK: {distance:.2f}m')
            # If we were paused and person is back in range, resume
            if self.nav_goal_handle is not None:
                # Navigation is already running, no need to do anything
                pass

    def pause_navigation(self):
        """일시 정지 - cmd_vel을 0으로 발행"""
        stop_cmd = Twist()  # All zeros
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Navigation PAUSED - waiting for person to return')

    def stop_navigation(self):
        """내비게이션 완전 중지"""
        if self.nav_goal_handle is not None:
            self.get_logger().info('Cancelling navigation goal...')
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            # Wait briefly for cancellation
            self.nav_goal_handle = None

        # Send stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.is_navigating = False

    def navigate_to_goal(self, goal_pose: PoseStamped):
        """목표 지점으로 로봇을 안내"""

        # Check if person is detected before starting
        if not self.is_person_detection_valid():
            self.get_logger().error('Cannot start navigation: No person detected!')
            return False

        self.get_logger().info(f'Starting navigation to goal: [{goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}]')

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return False

        # Create goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        # Send goal
        self.get_logger().info('Sending navigation goal...')
        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

        self.is_navigating = True
        return True

    def nav_goal_response_callback(self, future):
        """NavigateToPose goal response 콜백"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.is_navigating = False
            return

        self.get_logger().info('Navigation goal accepted!')
        self.nav_goal_handle = goal_handle

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """NavigateToPose result 콜백"""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation SUCCEEDED! Arrived at destination.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation CANCELED.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation ABORTED!')
        else:
            self.get_logger().info(f'Navigation finished with status: {status}')

        self.is_navigating = False
        self.nav_goal_handle = None


def main(args=None):
    rclpy.init(args=args)

    leading_server = LeadingServer()

    # Use MultiThreadedExecutor for action clients
    executor = MultiThreadedExecutor()
    executor.add_node(leading_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        leading_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
