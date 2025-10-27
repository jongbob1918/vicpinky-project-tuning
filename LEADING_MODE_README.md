# 🤖 Leading Mode (길안내 모드) 사용 가이드

로봇이 사람을 안내하면서 거리를 유지하고, 이탈 시 정지하는 기능입니다.

---

## 📦 구조

```
┌─────────────────────────────────────────────────┐
│  Astra Camera (항상 실행)                        │
│    ├─ /camera/color/image_raw                   │
│    └─ /camera/depth/image_raw                   │
│            ↓                                     │
│  Person Tracking Node (길안내 시에만 실행)       │
│    - YOLO + BoxMOT로 사람 감지 및 추적           │
│    - Depth로 거리 계산                           │
│    - /person_pose 발행                          │
│            ↓                                     │
│  Leading Server (길안내 시에만 실행)             │
│    - NavigateToPose로 목표로 이동                │
│    - /person_pose 구독하여 거리 모니터링         │
│    - 이탈 시 정지                                │
└─────────────────────────────────────────────────┘
```

---

## 🚀 설치 및 빌드

### 1. Python 패키지 설치
```bash
pip install ultralytics boxmot opencv-python
```

### 2. Re-ID 모델 다운로드
```bash
cd /home/mac/dev_ws/vicpinky-project-tuning
wget https://github.com/mikel-brostrom/yolov8_tracking/releases/download/v9.0/osnet_x0_25_msmt17.pt
```

### 3. 패키지 빌드
```bash
cd /home/mac/dev_ws/vicpinky-project-tuning
colcon build --packages-select vicpinky_navigation --symlink-install
source install/setup.bash
```

---

## 🎯 사용 방법

### 기본 사용법

#### Terminal 1: 로봇 + Nav2 실행
```bash
source /home/mac/dev_ws/vicpinky-project-tuning/install/setup.bash
ros2 launch vicpinky_navigation bringup_launch.xml
```

#### Terminal 2: Astra 카메라 실행
```bash
ros2 launch astra_camera dabai.launch.py
```

#### Terminal 3: 길안내 모드 시작 ⭐
```bash
source /home/mac/dev_ws/vicpinky-project-tuning/install/setup.bash
ros2 launch vicpinky_navigation leading_mode_launch.xml
```

#### Terminal 4: RViz에서 목표 지점 설정
```bash
ros2 run rviz2 rviz2
# "2D Goal Pose" 버튼으로 목표 지점 지정
```

---

## ⚙️ 파라미터 설정

### 1. 사람 추적 파라미터

launch 파일에서 변경:
```bash
ros2 launch vicpinky_navigation leading_mode_launch.xml \
    yolo_model:=yolov8n.pt \
    target_person_id:=1 \
    auto_select_closest:=true \
    show_debug_image:=true
```

또는 실행 중 변경:
```bash
# 추적할 사람 ID 변경
ros2 topic pub --once /person_tracking/target_id std_msgs/msg/Int32 "{data: 2}"

# 디버그 이미지 보기
ros2 run rqt_image_view rqt_image_view /person_tracking/debug_image
```

### 2. Leading Server 파라미터

`params/nav2_params.yaml`에서 수정:
```yaml
leading_server:
  desired_distance: 1.0        # 유지할 목표 거리 (m)
  min_person_distance: 0.5     # 최소 거리 (경고)
  max_person_distance: 2.0     # 최대 거리 (정지)
  detection_timeout: 3.0       # 타임아웃 (초)
```

실행 중 변경:
```bash
ros2 param set /leading_server max_person_distance 2.5
```

---

## 🔍 동작 확인

### 1. 사람 감지 확인
```bash
# person_pose가 발행되는지 확인
ros2 topic echo /person_pose

# 디버그 이미지 확인 (ID와 거리 표시)
ros2 run rqt_image_view rqt_image_view /person_tracking/debug_image
```

### 2. Leading 서버 로그 확인
```bash
ros2 topic echo /rosout | grep leading_server
```

---

## 📊 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/person_pose` | `PoseStamped` | 추적 중인 사람의 위치 |
| `/person_tracking/debug_image` | `Image` | 디버그용 트래킹 이미지 |
| `/person_tracking/target_id` | `Int32` | 추적할 사람 ID 설정 |
| `/goal_pose` | `PoseStamped` | 목표 지점 |

---

## 🎮 시나리오별 사용법

### 시나리오 1: 가장 가까운 사람 자동 추적
```bash
# auto_select_closest=true (기본값)
ros2 launch vicpinky_navigation leading_mode_launch.xml

# RViz에서 목표 지점 설정
# 로봇이 가장 가까운 사람을 자동으로 추적하며 목표로 이동
```

### 시나리오 2: 특정 ID 사람만 추적
```bash
# 먼저 사람들에게 ID 할당 (디버그 이미지로 확인)
ros2 launch vicpinky_navigation leading_mode_launch.xml \
    auto_select_closest:=false \
    target_person_id:=3 \
    show_debug_image:=true

# ID=3인 사람만 추적
```

### 시나리오 3: 사람 ID 동적 변경
```bash
# 처음엔 ID=1 추적
ros2 launch vicpinky_navigation leading_mode_launch.xml \
    auto_select_closest:=false \
    target_person_id:=1

# 중간에 ID=2로 변경
ros2 topic pub --once /person_tracking/target_id std_msgs/msg/Int32 "{data: 2}"
```

---

## 🧪 테스트 방법

### 시뮬레이션 테스트 (카메라 없이)
```bash
# Terminal 1: Nav2
ros2 launch vicpinky_navigation bringup_launch.xml

# Terminal 2: 테스트용 사람 위치 발행
ros2 run vicpinky_navigation test_person_publisher.py

# Terminal 3: Leading Server만 실행
ros2 run vicpinky_navigation leading_server.py

# 사람 거리 변경 테스트
ros2 param set /test_person_publisher person_distance 3.0  # 이탈
ros2 param set /test_person_publisher person_distance 1.0  # 복귀
```

---

## ⚠️ 문제 해결

### 1. "Person detection lost!" 계속 표시
```bash
# 원인: 카메라 토픽이 없거나 사람이 감지되지 않음

# 해결 1: 카메라 토픽 확인
ros2 topic list | grep camera

# 해결 2: 사람 감지 확인 (디버그 이미지)
ros2 launch vicpinky_navigation leading_mode_launch.xml show_debug_image:=true

# 해결 3: YOLO 모델 확인
ls -l yolov8n.pt osnet_x0_25_msmt17.pt
```

### 2. "NavigateToPose action server not available!"
```bash
# 원인: Nav2가 실행 중이지 않음

# 해결: Nav2 실행 확인
ros2 node list | grep bt_navigator

# 없으면 Nav2 재시작
ros2 launch vicpinky_navigation bringup_launch.xml
```

### 3. BoxMOT/YOLO import 에러
```bash
# 원인: Python 패키지 미설치

# 해결:
pip install ultralytics boxmot opencv-python
```

### 4. 로봇이 계속 멈춤
```bash
# 원인: max_person_distance가 너무 작음

# 해결: 거리 늘리기
ros2 param set /leading_server max_person_distance 3.0

# 또는 yaml 파일 수정
# params/nav2_params.yaml:
#   leading_server:
#     max_person_distance: 3.0
```

---

## 🔧 커스터마이징

### YOLO 모델 변경 (자신의 모델 사용)
```bash
# 학습한 모델 사용
ros2 launch vicpinky_navigation leading_mode_launch.xml \
    yolo_model:=/path/to/your/model.pt
```

### 좌표계 변환 수정
`person_tracking_node.py`에서 수정:
```python
# 카메라 → 로봇 좌표계 변환 (line 200-205)
pose_msg.pose.position.x = person_z / 1000.0  # forward
pose_msg.pose.position.y = -person_x / 1000.0  # left
pose_msg.pose.position.z = -person_y / 1000.0  # up
```

---

## 📝 핵심 차이점 정리

### 이전 vs 현재

| 항목 | 이전 | 현재 |
|------|------|------|
| **following_server** | ✅ 포함 (로봇이 따라감) | ❌ 제거 |
| **leading_server** | ✅ 항상 실행 | ✅ 필요할 때만 실행 |
| **사람 감지** | ❌ 없음 | ✅ BoxMOT + YOLO |
| **카메라 통합** | ❌ 없음 | ✅ Astra Camera |
| **실행 방법** | bringup으로 자동 | leading_mode_launch로 별도 |

---

## 📚 파일 구조

```
vicpinky_navigation/
├── scripts/
│   ├── leading_server.py              # 거리 유지 및 이탈 감지
│   ├── person_tracking_node.py        # BoxMOT 사람 추적 ⭐
│   ├── person_detector_node.py        # MediaPipe 대안
│   ├── test_person_publisher.py       # 테스트용
│   └── test_leading_client.py         # 테스트용
├── launch/
│   ├── bringup_launch.xml             # Nav2 기본 실행
│   ├── navigation_launch.xml          # Nav2 노드들
│   └── leading_mode_launch.xml        # 길안내 모드 ⭐
└── params/
    └── nav2_params.yaml               # 모든 파라미터
```

---

## 💡 다음 단계

1. **실제 환경 테스트**
   - Astra 카메라로 사람 감지 확인
   - 거리 유지 동작 확인
   - 이탈 → 정지 → 복귀 시나리오 테스트

2. **파라미터 튜닝**
   - max_person_distance 조정
   - detection_timeout 조정
   - YOLO confidence 조정

3. **개선 사항** (선택)
   - Action Server로 변경
   - 자동 재개 로직 추가
   - 속도 조절 기능 추가

---

## 📞 문의

문제가 있으면 로그를 확인하세요:
```bash
ros2 topic echo /rosout | grep -E "(person_tracking|leading_server)"
```
