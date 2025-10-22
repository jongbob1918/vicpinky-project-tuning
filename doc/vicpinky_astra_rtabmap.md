# vicpinky + Orbbec Astra + RTAB-Map 개발 진행 문서

이 문서는 실물 Vic Pinky 로봇에 Orbbec Astra RGB-D 카메라를 장착해 RTAB-Map 기반 Visual SLAM 워크플로를 구축하기 위한 진행 현황과 실행 단계를 정리한 것이다.

## 1. 진행 현황
- [x] 워크스페이스 구조 파악 (`README.md`, `vicpinky_*`, `ros2_astra_camera`, `rtabmap_ros-jazzy-devel`)
- [x] 기존 문서 검토 (`doc/lidar_setup.md`, `ros2_astra_camera/doc/depth_camera_setup.md`)
- [x] 실물 로봇에 Astra 장착 및 좌표계 측정 (2025.10.22 완료)
- [x] URDF/TF에 카메라 링크 반영 (robot_core.xacro, camera.xacro 수정 완료)
- [x] bringup 런치 확장 및 Astra 드라이버 통합 (2025.10.22 완료)
- [x] RTAB-Map RGB-D+LIDAR SLAM 런치 구성 (2025.10.22 완료)
- [ ] Nav2 + RTAB-Map 시나리오 통합 및 검증

## 2. 목표 요약
- Vic Pinky 로봇 상단 폴대에 Orbbec Astra를 장착하여 RGB-D 데이터를 수집한다.
- LIDAR 기반 흐름과 병행 가능한 RTAB-Map Visual SLAM 파이프라인을 구축한다.
- Nav2 내비게이션에서 RTAB-Map이 생성한 지도/오도메트리를 활용할 수 있도록 한다.

## 3. 하드웨어 구성 가이드
- **카메라 장착**: 폴대 높이를 충분히 확보하고 카메라의 수평을 맞춘 상태에서 base_link 기준 위치/방향을 실측.
- **케이블/전원**: USB 3.0 케이블 길이 및 고정, 5V 1A 이상 전원 또는 유전원 허브 권장.
- **간섭 최소화**: LIDAR와 시야 간섭이 없도록 배치하고, RViz에서 깊이 이미지로 시각 점검.

## 4. 소프트웨어 준비
### 4.1 기본 워크스페이스
- `README.md:8` 지침을 따라 Jazzy 기반 워크스페이스를 구성하고 `vicpinky_*` 패키지를 빌드.
- `doc/99-vic-pinky.rules`로 udev 설정 후 `sllidar_ros2`를 설치(`doc/lidar_setup.md` 참고).

### 4.2 Orbbec Astra 드라이버
- `ros2_astra_camera/doc/depth_camera_setup.md`에 따라 필수 시스템 패키지, `libuvc`, `magic_enum`을 설치.
- `ros2 launch astra_camera astra_mini.launch.py`로 토픽(`/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth/color/points`) 확인.
- udev 스크립트(`ros2_astra_camera/astra_camera/scripts/install.sh`) 실행 후 재로그인.

### 4.3 RTAB-Map 소스
- 워크스페이스에 `rtabmap_ros-jazzy-devel`과 `rtabmap-0.22.1-jazzy`가 포함되어 있으므로 `colcon build --symlink-install --packages-select rtabmap_ros`로 필요한 노드만 빌드 가능.
- DDS는 CycloneDDS 사용을 권장(`rtabmap_ros-jazzy-devel/README.md:70`).

## 5. 통합 로드맵
### 단계 1: 센서 모델링 및 TF 정리
- `vicpinky_description/urdf/robot_core.xacro`에 `camera_mount` 및 `camera_link` 링크/조인트 추가.
- 실제 장착 위치를 반영한 `base_link → camera_link` 변환을 URDF 또는 `static_transform_publisher`로 퍼블리시.
- Gazebo 전용 `camera.xacro`는 시뮬레이션용이므로 실물용 링크 정의를 별도로 작성.

### 단계 2: bringup 런치 확장
- `vicpinky_bringup/launch/bringup.launch.xml`을 기반으로 카메라 드라이버를 포함하는 새 런치(예: `bringup_rgbd.launch.xml`) 작성.
- LIDAR 사용 여부를 인자로 제어하여 Visual SLAM 단독 테스트 시 레이저 노드를 비활성화할 수 있도록 설계.
- `laser_filters` 대신 RGB-D 기반 파이프라인 사용 시 `/scan_filtered` 의존성을 제거하거나 조건부로 유지.

### 단계 3: RTAB-Map RGB-D 오도메트리/SLAM 구성
- `rtabmap_launch/launch/rtabmap.launch.py`를 include하여 RGB-D 오도메트리(`rtabmap_odom`)와 SLAM(`rtabmap_slam`)을 기동.
- 주요 인자:
  - `frame_id:=base_link`
  - `odom_frame:=odom_rgbd` (또는 Nav2가 사용하는 `odom`)
  - Image/Depth 토픽 remap: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/depth/camera_info`
  - 2D 구동 제약: `Reg/Force3DoF:=true`, `RGBD/OptimizeFromGraphEnd:=false`
- RViz 설정은 `rtabmap_ros-jazzy-devel/rtabmap_examples/config/` 내 샘플을 참고.

### 단계 4: Nav2 통합
- RTAB-Map의 `/rtabmap/odom` 또는 `/rtabmap/localization_pose`를 Nav2에서 사용하는 `odom` 트리로 연결.
- 기존 slam_toolbox 기반 맵 생성 흐름과 충돌하지 않도록 launch 인자를 통해 Visual SLAM / LIDAR SLAM을 선택 가능하게 구성.
- 필요 시 `robot_localization` EKF로 바퀴 오도메트리와 RGB-D 오도메트리를 융합.

### 단계 5: 튜닝 및 검증
- RTAB-Map 파라미터 튜닝: `Vis/MinInliers`, `RGBD/AngularUpdate`, `RGBD/LinearUpdate`, `LoopClosure/Strategy`.
- DDS QoS: RViz/rtabmap_ros에서 sensor_data QoS(=Best Effort) 사용 여부 확인.
- 반복적으로 맵 품질과 루프 클로저 빈도를 점검하면서 파라미터를 조정.

## 6. 테스트 체크리스트
- [ ] `ros2 topic list`에서 카메라 토픽과 TF(`/tf`, `/tf_static`)가 정상 노출되는지 확인.
- [ ] `ros2 run tf2_tools view_frames` 결과에서 `base_link -> camera_link`가 중복 없이 존재하는지 확인.
- [ ] RTAB-Map 실행 시 `/rtabmap/info`에서 loop closure, 메모리 사용량, processing time 확인.
- [ ] Nav2 bringup 후 목표 지점까지 경로 생성 및 추종 여부 확인.
- [ ] `nav2_map_server`로 저장한 맵과 실제 환경의 스케일이 일치하는지 검증.

## 7. 참고 경로
- 기본 세팅: `README.md`
- LIDAR 설정: `doc/lidar_setup.md`
- Astra 드라이버 안내: `ros2_astra_camera/doc/depth_camera_setup.md`
- 로봇 모델: `vicpinky_description/urdf/robot_core.xacro`, `vicpinky_description/urdf/robot.urdf.xacro`
- Bringup: `vicpinky_bringup/launch/bringup.launch.xml`
- RTAB-Map 예제/파라미터: `rtabmap_ros-jazzy-devel/rtabmap_examples/*`, `rtabmap_ros-jazzy-devel/rtabmap_launch/launch/rtabmap.launch.py`

## 8. 카메라 실측 정보 (2025.10.22 적용)
- **카메라 위치**: laser_mount 기준 X축 -29cm, Z축 109cm
- **base_link 기준**: xyz="-0.29 0 1.27" (laser_mount가 base_link에서 z=0.18에 위치)
- **방향**: 정면(전방) 향하도록 설정 (rpy="0 0 0")
- **URDF 적용**: `robot_core.xacro`에 camera_mount, camera_link 추가 완료
- **Gazebo 모델**: Orbbec Astra 사양에 맞춰 RGB/Depth 센서 설정 완료

## 9. 미해결 사항 및 메모
- [x] Astra 장착 좌표 실측 후 URDF 패치 완료 (2025.10.22)
- Visual SLAM 성능 검증을 위해 실내 주행 로그 수집 예정.
- Nav2 통합 시 2D costmap과 RGB-D 포인트클라우드로 장애물 업데이트를 수행할지 선택 필요.
