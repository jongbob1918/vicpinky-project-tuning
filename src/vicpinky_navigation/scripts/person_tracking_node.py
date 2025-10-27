#!/usr/bin/env python3
"""
Person Tracking Node with BoxMOT + YOLO + Astra Camera
사용자의 기존 BoxMOT 트래킹 코드를 Astra 카메라와 연결

Requirements:
    pip install ultralytics boxmot opencv-python
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path

try:
    from ultralytics import YOLO
    from boxmot import StrongSort
    BOXMOT_AVAILABLE = True
except ImportError:
    BOXMOT_AVAILABLE = False


class PersonTrackingNode(Node):
    """
    BoxMOT + YOLO를 사용한 사람 트래킹 노드
    - Astra Camera RGB + Depth 사용
    - 특정 ID 사람을 추적
    - /person_pose 발행
    """

    def __init__(self):
        super().__init__('person_tracking_node')

        # Parameters
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('yolo_model', 'yolov8n.pt')  # or your trained model
        self.declare_parameter('reid_model', 'osnet_x0_25_msmt17.pt')
        self.declare_parameter('target_person_id', 1)  # 추적할 사람 ID (기본: 1번)
        self.declare_parameter('auto_select_closest', True)  # True면 가장 가까운 사람 자동 선택
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('show_debug_image', False)  # 디버그 이미지 표시 여부

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        yolo_model_path = self.get_parameter('yolo_model').value
        reid_model_path = self.get_parameter('reid_model').value
        self.target_person_id = self.get_parameter('target_person_id').value
        self.auto_select_closest = self.get_parameter('auto_select_closest').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.show_debug = self.get_parameter('show_debug_image').value

        if not BOXMOT_AVAILABLE:
            self.get_logger().error('BoxMOT or YOLO not available!')
            self.get_logger().error('Install: pip install ultralytics boxmot')
            return

        # CV Bridge
        self.bridge = CvBridge()

        # Data storage
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None

        # YOLO + BoxMOT 초기화
        try:
            self.get_logger().info(f'Loading YOLO model: {yolo_model_path}')
            self.model = YOLO(yolo_model_path)

            self.get_logger().info(f'Loading Re-ID model: {reid_model_path}')
            self.tracker = StrongSort(
                reid_weights=Path(reid_model_path),
                device='cpu',
                half=False,
                max_age=1200,
                cmc_method="disable"
            )
            self.get_logger().info('YOLO + StrongSort initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize YOLO/BoxMOT: {e}')
            return

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

        # Subscriber for target person ID change
        self.target_id_sub = self.create_subscription(
            Int32, '/person_tracking/target_id', self.target_id_callback, 10
        )

        # Publishers
        self.person_pose_pub = self.create_publisher(PoseStamped, '/person_pose', 10)
        self.debug_image_pub = self.create_publisher(Image, '/person_tracking/debug_image', 10)

        # Timer for processing (10Hz)
        self.timer = self.create_timer(0.1, self.process_frame)

        self.get_logger().info('Person Tracking Node started')
        self.get_logger().info(f'  RGB Topic: {rgb_topic}')
        self.get_logger().info(f'  Depth Topic: {depth_topic}')
        self.get_logger().info(f'  Target Person ID: {self.target_person_id}')
        self.get_logger().info(f'  Auto Select Closest: {self.auto_select_closest}')

    def rgb_callback(self, msg: Image):
        """RGB 이미지 수신"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert RGB image: {e}')

    def depth_callback(self, msg: Image):
        """Depth 이미지 수신"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert Depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """카메라 정보 수신"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera info received')

    def target_id_callback(self, msg: Int32):
        """추적할 사람 ID 변경"""
        self.target_person_id = msg.data
        self.get_logger().info(f'Target person ID changed to: {self.target_person_id}')

    def process_frame(self):
        """프레임 처리 및 사람 추적"""

        if self.latest_rgb is None or self.latest_depth is None:
            return

        frame = self.latest_rgb.copy()

        # YOLO 객체 탐지 (class 0 = person)
        results = self.model(frame, classes=[0], verbose=False)

        # BoxMOT 추적기 업데이트
        tracks = self.tracker.update(results[0].boxes.data.cpu().numpy(), frame)

        # 추적 결과 처리
        if tracks.shape[0] > 0:
            target_track = None

            if self.auto_select_closest:
                # 가장 가까운 사람 선택
                target_track = self.select_closest_person(tracks)
            else:
                # 특정 ID 찾기
                for track in tracks:
                    track_id = int(track[4])
                    if track_id == self.target_person_id:
                        target_track = track
                        break

            # 타겟 사람이 있으면 위치 발행
            if target_track is not None:
                self.publish_person_pose(target_track)

            # 디버그 이미지 생성
            if self.show_debug or self.debug_image_pub.get_subscription_count() > 0:
                self.draw_tracking_results(frame, tracks, target_track)

    def select_closest_person(self, tracks):
        """가장 가까운 사람 선택 (depth 기준)"""
        min_distance = float('inf')
        closest_track = None

        for track in tracks:
            x1, y1, x2, y2, track_id = map(int, track[:5])

            # 바운딩 박스 중심
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Depth 값 가져오기
            depth_value = self.get_depth_at_pixel(center_x, center_y)

            if depth_value is not None and depth_value > 0:
                distance = depth_value / 1000.0  # mm to m
                if distance < min_distance:
                    min_distance = distance
                    closest_track = track

        return closest_track

    def publish_person_pose(self, track):
        """사람 위치 발행"""
        x1, y1, x2, y2, track_id = map(int, track[:5])

        # 바운딩 박스 중심
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2

        # Depth 값 가져오기
        depth_value = self.get_depth_at_pixel(center_x, center_y)

        if depth_value is not None and depth_value > 0:
            # 3D 좌표 계산
            person_x, person_y, person_z = self.pixel_to_3d(center_x, center_y, depth_value)

            # PoseStamped 발행
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.camera_frame

            # 카메라 → 로봇 좌표계 변환
            # Astra: z=depth(forward), x=right, y=down
            # Robot: x=forward, y=left, z=up
            pose_msg.pose.position.x = person_z / 1000.0  # mm to m, forward
            pose_msg.pose.position.y = -person_x / 1000.0  # mm to m, left
            pose_msg.pose.position.z = -person_y / 1000.0  # mm to m, up
            pose_msg.pose.orientation.w = 1.0

            self.person_pose_pub.publish(pose_msg)

            # Log
            self.get_logger().info(
                f'Person ID={track_id}: x={pose_msg.pose.position.x:.2f}m, '
                f'y={pose_msg.pose.position.y:.2f}m',
                throttle_duration_sec=1.0
            )

    def draw_tracking_results(self, frame, tracks, target_track):
        """디버그용 트래킹 결과 그리기"""
        for track in tracks:
            x1, y1, x2, y2, track_id = map(int, track[:5])

            # 타겟 사람은 초록색, 나머지는 파란색
            if target_track is not None and track_id == int(target_track[4]):
                color = (0, 255, 0)  # Green
                thickness = 3
            else:
                color = (255, 0, 0)  # Blue
                thickness = 2

            # 바운딩 박스 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

            # ID와 거리 표시
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            depth_value = self.get_depth_at_pixel(center_x, center_y)

            if depth_value is not None:
                distance = depth_value / 1000.0  # mm to m
                text = f"ID: {track_id}, {distance:.2f}m"
            else:
                text = f"ID: {track_id}"

            cv2.putText(frame, text, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # ROS2 토픽으로 발행
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')

        # 화면 표시 (옵션)
        if self.show_debug:
            cv2.imshow("Person Tracking", frame)
            cv2.waitKey(1)

    def get_depth_at_pixel(self, x, y, window_size=5):
        """픽셀 위치의 depth 값 가져오기"""
        if self.latest_depth is None:
            return None

        h, w = self.latest_depth.shape[:2]
        half_window = window_size // 2

        x_min = max(0, x - half_window)
        x_max = min(w, x + half_window + 1)
        y_min = max(0, y - half_window)
        y_max = min(h, y + half_window + 1)

        depth_window = self.latest_depth[y_min:y_max, x_min:x_max]
        valid_depths = depth_window[depth_window > 0]

        if len(valid_depths) > 0:
            return np.median(valid_depths)
        return None

    def pixel_to_3d(self, u, v, depth):
        """픽셀 → 3D 좌표 변환"""
        if self.camera_info is None:
            fx, fy = 570.0, 570.0  # Astra 기본값
            cx, cy = 320.0, 240.0
        else:
            K = self.camera_info.k
            fx, fy = K[0], K[4]
            cx, cy = K[2], K[5]

        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return x, y, z


def main(args=None):
    rclpy.init(args=args)

    node = PersonTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_debug:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
