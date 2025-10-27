# Leading Mode Guide
로봇이 사람을 안내(leading)하면서 거리를 유지하는 기능 사용 가이드

## 📋 개요

**Leading Server**는 로봇이 목표 지점으로 이동하면서 뒤따라오는 사람과 일정 거리를 유지하고, 사람이 이탈하면 정지하는 기능입니다.

### 주요 기능
- ✅ 로봇이 목표 지점으로 안내
- ✅ 사람과 일정 거리 유지 (기본: 1m)
- ✅ 사람 이탈 감지 시 자동 정지
- ✅ 사람 복귀 시 재개
- ✅ 스테레오/뎁스 카메라 지원

---

## 🚀 빌드 및 설치

```bash
cd /home/mac/dev_ws/vicpinky-project-tuning
colcon build --packages-select vicpinky_navigation
source install/setup.bash
```

---

## ⚙️ 설정

### 파라미터 (`nav2_params.yaml`)

```yaml
leading_server:
  ros__parameters:
    controller_frequency: 10.0        # 모니터링 주기 (Hz)
    desired_distance: 1.0             # 유지할 목표 거리 (m)
    min_person_distance: 0.5          # 최소 거리 (너무 가까우면 경고)
    max_person_distance: 2.0          # 최대 거리 (이탈 판정)
    detection_timeout: 3.0            # 사람 감지 타임아웃 (초)
    base_frame: "base_footprint"
    person_pose_topic: "/person_pose"
```

---

## 🎯 사용 방법

### 1. Nav2 + Leading Server 실행

```bash
# 전체 네비게이션 스택 실행 (leading_server 포함)
ros2 launch vicpinky_navigation bringup_launch.xml
```

### 2. 사람 위치 발행 (뎁스 카메라)

실제 환경에서는 **뎁스 카메라 노드**에서 사람 위치를 발행해야 합니다:

```python
# 당신의 카메라 노드에서
person_pose = PoseStamped()
person_pose.header.frame_id = "base_footprint"  # 또는 camera_link
person_pose.header.stamp = self.get_clock().now().to_msg()
person_pose.pose.position.x = person_x  # 뎁스 정보
person_pose.pose.position.y = person_y
person_pose.pose.position.z = person_z
person_pose_pub.publish(person_pose)
```

### 3. 목표 지점 전송

**Option A: RViz2 사용**
1. RViz2에서 `2D Goal Pose` 버튼 클릭
2. 지도에서 목표 지점 지정

**Option B: 명령줄 사용**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

**Option C: Python으로 호출**
```python
from geometry_msgs.msg import PoseStamped

# leading_server 노드의 navigate_to_goal() 호출
goal_pose = PoseStamped()
goal_pose.header.frame_id = "map"
goal_pose.pose.position.x = 2.0
goal_pose.pose.position.y = 1.0
leading_server.navigate_to_goal(goal_pose)
```

---

## 🧪 테스트 방법

### 테스트 1: 시뮬레이션 (사람 위치 발행기 사용)

```bash
# Terminal 1: Nav2 + Leading Server
ros2 launch vicpinky_navigation bringup_launch.xml

# Terminal 2: 테스트용 사람 위치 발행
ros2 run vicpinky_navigation test_person_publisher.py

# Terminal 3: RViz로 목표 지점 설정
rviz2 -d $(ros2 pkg prefix vicpinky_navigation)/share/vicpinky_navigation/rviz/nav2_view.rviz
```

### 테스트 2: 사람 거리 변경하기

```bash
# 사람을 1.5m 떨어뜨리기
ros2 param set /test_person_publisher person_distance 1.5

# 사람을 3.0m로 이탈시키기 (정지 확인)
ros2 param set /test_person_publisher person_distance 3.0

# 사람을 다시 1.0m로 복귀시키기 (재개 확인)
ros2 param set /test_person_publisher person_distance 1.0
```

---

## 📊 동작 로직

```
┌─────────────────────────────────────────────────────────┐
│  1. 목표 지점 수신 (NavigateToPose)                      │
│     ↓                                                     │
│  2. 사람 감지 확인 (/person_pose)                        │
│     ↓                                                     │
│  3. 내비게이션 시작                                       │
│     ↓                                                     │
│  4. 주기적 모니터링 (10Hz)                               │
│     ├─ 사람 거리 > max_distance (2.0m)?                 │
│     │  ├─ YES → 정지 (pause_navigation)                 │
│     │  └─ NO  → 계속                                     │
│     ├─ 사람 거리 < min_distance (0.5m)?                 │
│     │  └─ YES → 경고 (로그)                              │
│     └─ 감지 타임아웃 (3.0s)?                            │
│        └─ YES → 완전 정지 (stop_navigation)             │
│                                                           │
│  5. 목표 도착 또는 취소                                   │
└─────────────────────────────────────────────────────────┘
```

---

## 🔧 실제 통합 방법

### 뎁스 카메라 노드 수정

당신의 스테레오/뎁스 카메라 노드에서 다음과 같이 수정:

```python
class DepthCameraNode(Node):
    def __init__(self):
        super().__init__('depth_camera_node')

        # Person pose publisher 추가
        self.person_pose_pub = self.create_publisher(
            PoseStamped, '/person_pose', 10
        )

    def process_depth_image(self, depth_image):
        # 사람 감지 로직
        person_detected, person_x, person_y, person_z = self.detect_person(depth_image)

        if person_detected:
            # PoseStamped로 발행
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_link"  # 또는 base_footprint
            pose_msg.pose.position.x = person_x
            pose_msg.pose.position.y = person_y
            pose_msg.pose.position.z = person_z
            pose_msg.pose.orientation.w = 1.0

            self.person_pose_pub.publish(pose_msg)
```

---

## 📝 TODO / 개선 사항

현재 `leading_server.py`는 기본 구현입니다. 다음 개선이 필요합니다:

1. **Action Server 구현**
   - 현재: 직접 `navigate_to_goal()` 호출 필요
   - 개선: Action Server로 만들어 ROS2 Action 사용

2. **목표 지점 토픽 구독**
   - 현재: 코드에서 직접 호출
   - 개선: `/goal_pose` 토픽 구독하여 자동 시작

3. **속도 조절**
   - 현재: 정지만 가능
   - 개선: 사람이 가까우면 속도 감소

4. **재개 로직 개선**
   - 현재: pause 후 자동 재개 없음
   - 개선: 사람 복귀 시 NavigateToPose 재호출

---

## 🐛 문제 해결

### 1. "Person detection lost!" 계속 표시
```bash
# 사람 위치가 발행되고 있는지 확인
ros2 topic echo /person_pose

# 발행 안되면 카메라 노드 확인
ros2 node list | grep camera
```

### 2. "NavigateToPose action server not available!"
```bash
# Nav2가 실행 중인지 확인
ros2 node list | grep bt_navigator

# 없으면 Nav2 재시작
ros2 launch vicpinky_navigation bringup_launch.xml
```

### 3. 로봇이 계속 멈춤
```bash
# 사람 거리 파라미터 확인
ros2 param get /leading_server max_person_distance

# 값을 늘리기
ros2 param set /leading_server max_person_distance 3.0
```

---

## 📚 관련 파일

- **노드**: `scripts/leading_server.py`
- **설정**: `params/nav2_params.yaml` (leading_server 섹션)
- **런치**: `launch/navigation_launch.xml`
- **테스트**: `scripts/test_person_publisher.py`

---

## 📧 문의

문제가 있거나 개선 제안이 있으면 이슈를 등록해주세요.
