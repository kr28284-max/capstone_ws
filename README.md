# AI 비전 기반 지능형 부품 분류 시스템 (2026 Capstone Design)

본 프로젝트는 **YOLO v8**과 **Intel RealSense** 카메라를 활용하여 실시간으로 부품을 인식하고, **ROS 2** 및 **MoveIt 2**를 통해 **OpenManipulator-X** 로봇 팔을 제어하여 부품을 자동으로 분류하는 시스템입니다.

## 🌟 주요 특징
- **AI 객체 인식:** YOLO 모델을 이용해 4종 부품(Bearing, Boltnut, Gear, Wheel) 실시간 탐지
- **6D Pose 스타일 좌표 변환:** 카메라의 Depth 정보와 Optical Frame을 로봇의 `base_link` 좌표계로 변환
- **지능형 경로 계획:** MoveIt 2를 활용한 정밀한 Pick & Place 및 장애물 회피(Constraints 반영)
- **멀티스레드 제어:** 비전 수신과 로봇 제어의 병렬 처리를 통한 실시간성 확보

## 🛠 하드웨어 구성
- **Robot:** TurtleBot 3 OpenManipulator-X
- **Sensor:** Intel RealSense D435 (또는 호환 카메라)
- **Controller:** PC (Ubuntu 22.04 / ROS 2 Humble 권장)

## 📂 패키지 및 노드 설명

### 1. `vision_pkg` (`vision_master_node5.py`)
- **역할:** 비전 인식 및 좌표 계산 서버
- **주요 기능:**
  - RealSense RGB-D 정렬(Align) 및 Depth 데이터 기반 3D 좌표 추출
  - 카메라 좌표계 -> `base_link` 좌표계 변환 (오프셋 보정 포함)
  - `/vision/detections`: 인식된 모든 물체의 정보를 JSON 형식으로 발행
  - 실시간 GUI를 통해 인식 결과(Bounding Box, XYZ 좌표, FPS) 시각화

### 2. `control_pkg` (`test_node3.py`)
- **역할:** 로봇 팔 동작 제어 및 시퀀스 관리
- **주요 기능:**
  - `/pick_command`: 입력된 명령(1~4)에 따라 특정 클래스 부품 분류 시작
  - **Leftmost 전략:** 동일 클래스가 여러 개일 경우 가장 왼쪽에 있는 물체부터 우선 처리
  - **정밀 보정:** 원거리 물체에 대한 Z축 높이 보정 및 그리퍼 수직 유지 제약(Orientation Constraint) 적용
  - 인식 자세(Vision Pose)와 투입 위치(Drop Zone) 간 자동 이동

## 🚥 통신 프로토콜 (Topic)

| Topic 명 | 타입 | 설명 |
| :--- | :--- | :--- |
| `/vision/detections` | `std_msgs/String` | 인식된 객체들의 리스트 (JSON: class, x, y, z 등) |
| `/pick_command` | `std_msgs/Int32` | 작업 명령 (1:Bearing, 2:Boltnut, 3:Gear, 4:Wheel) |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 관절 각도 기반 직접 제어 |

## 🚀 실행 방법

### 환경 설정
```bash
# 워크스페이스 빌드
cd ~/capstone_ws
colcon build --symlink-install
source install/setup.bash
