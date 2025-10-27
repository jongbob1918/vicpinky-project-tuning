#!/usr/bin/env python3
"""
Person Detection Node for Astra Camera
RGB 이미지에서 사람을 감지하고 Depth 정보로 거리를 계산하여 /person_pose 발행

Requirements:
    pip install opencv-python mediapipe
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False


class PersonDetectorNode(Node):
    """
    Astra 카메라에서 사람을 감지하고 위치를 발행하는 노드
    - RGB: /camera/color/image_raw
    - Depth: /camera/depth/image_raw
    - Output: /person_pose
    """

    def __init__(self):
        super().__init__('person_detector_node')

        # Parameters
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_link')

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.detection_confidence = self.get_parameter('detection_confidence').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # CV Bridge
        self.bridge = CvBridge()

        # Data storage
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None

        # MediaPipe Pose Detection
        if MEDIAPIPE_AVAILABLE:
            self.mp_pose = mp.solutions.pose
            self.pose_detector = self.mp_pose.Pose(
                static_image_mode=False,
                model_complexity=1,
                min_detection_confidence=self.detection_confidence,
                min_tracking_confidence=0.5
            )
            self.get_logger().info('MediaPipe initialized successfully')
        else:
            self.get_logger().error('MediaPipe not available! Install: pip install mediapipe')
            self.pose_detector = None

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )

        # Publisher
        self.person_pose_pub = self.create_publisher(PoseStamped, '/person_pose', 10)

        # Timer for processing
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.detect_and_publish)

        self.get_logger().info('Person Detector Node started')
        self.get_logger().info(f'  RGB Topic: {rgb_topic}')
        self.get_logger().info(f'  Depth Topic: {depth_topic}')
        self.get_logger().info(f'  Detection Confidence: {self.detection_confidence}')

    def rgb_callback(self, msg: Image):
        """RGB 이미지 수신"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert RGB image: {e}')

    def depth_callback(self, msg: Image):
        """Depth 이미지 수신"""
        try:
            # Depth는 보통 16UC1 또는 32FC1
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert Depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """카메라 정보 수신 (한 번만)"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera info received')

    def detect_and_publish(self):
        """사람 감지 및 위치 발행"""

        if self.latest_rgb is None or self.latest_depth is None:
            return

        if not MEDIAPIPE_AVAILABLE or self.pose_detector is None:
            return

        # RGB를 RGB로 변환 (MediaPipe는 RGB 사용)
        rgb_image = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2RGB)

        # MediaPipe로 사람 감지
        results = self.pose_detector.process(rgb_image)

        if results.pose_landmarks:
            # 사람의 중심점 계산 (어깨 중심 또는 몸통 중심)
            landmarks = results.pose_landmarks.landmark

            # 어깨 중심점 사용 (left_shoulder, right_shoulder)
            left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]

            # 이미지 좌표로 변환
            h, w = self.latest_rgb.shape[:2]
            center_x = int((left_shoulder.x + right_shoulder.x) / 2.0 * w)
            center_y = int((left_shoulder.y + right_shoulder.y) / 2.0 * h)

            # 유효 범위 체크
            if 0 <= center_x < w and 0 <= center_y < h:
                # Depth 값 가져오기 (5x5 영역의 중앙값 사용)
                depth_value = self.get_depth_at_pixel(center_x, center_y)

                if depth_value is not None and depth_value > 0:
                    # 3D 좌표 계산 (카메라 좌표계)
                    person_x, person_y, person_z = self.pixel_to_3d(
                        center_x, center_y, depth_value
                    )

                    # PoseStamped 발행
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = self.camera_frame

                    # 카메라 좌표계: z는 앞, x는 오른쪽, y는 아래
                    # 로봇 좌표계로 변환: x는 앞, y는 왼쪽, z는 위
                    pose_msg.pose.position.x = person_z / 1000.0  # mm to m
                    pose_msg.pose.position.y = -person_x / 1000.0  # mm to m
                    pose_msg.pose.position.z = -person_y / 1000.0  # mm to m
                    pose_msg.pose.orientation.w = 1.0

                    self.person_pose_pub.publish(pose_msg)

                    # Log occasionally
                    if self.get_clock().now().nanoseconds % 2_000_000_000 < 200_000_000:
                        self.get_logger().info(
                            f'Person detected at: x={pose_msg.pose.position.x:.2f}m, '
                            f'y={pose_msg.pose.position.y:.2f}m, '
                            f'z={pose_msg.pose.position.z:.2f}m',
                            throttle_duration_sec=2.0
                        )

    def get_depth_at_pixel(self, x, y, window_size=5):
        """픽셀 위치의 depth 값 가져오기 (주변 평균)"""
        if self.latest_depth is None:
            return None

        h, w = self.latest_depth.shape[:2]
        half_window = window_size // 2

        # 영역 설정
        x_min = max(0, x - half_window)
        x_max = min(w, x + half_window + 1)
        y_min = max(0, y - half_window)
        y_max = min(h, y + half_window + 1)

        # 윈도우 내 유효한 depth 값들의 중앙값
        depth_window = self.latest_depth[y_min:y_max, x_min:x_max]
        valid_depths = depth_window[depth_window > 0]

        if len(valid_depths) > 0:
            return np.median(valid_depths)
        return None

    def pixel_to_3d(self, u, v, depth):
        """픽셀 좌표를 3D 좌표로 변환"""
        if self.camera_info is None:
            # 기본 Astra 파라미터 (근사값)
            fx = 570.0  # focal length x
            fy = 570.0  # focal length y
            cx = 320.0  # principal point x
            cy = 240.0  # principal point y
        else:
            K = self.camera_info.k
            fx = K[0]
            fy = K[4]
            cx = K[2]
            cy = K[5]

        # 3D 좌표 계산
        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return x, y, z


def main(args=None):
    rclpy.init(args=args)

    node = PersonDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
