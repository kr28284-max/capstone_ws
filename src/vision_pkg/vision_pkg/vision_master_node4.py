import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs
from ultralytics import YOLO
import cv2
import numpy as np
import threading
import time

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.CAMERA_HEIGHT = 0.304

        # 1. 모델 로드 및 CPU 최적화 설정
        # 가중치 파일 경로는 본인의 환경에 맞춰 수정하세요.
        self.model = YOLO("/home/user/capstone_ws/best.pt")
        self.class_names = self.model.names
        self.prev_time = 0
        
        # 2. 리얼센스 파이프라인 설정 (노트북 직결 모드)
        self.pipeline = rs.pipeline()
        config = rs.config()
        # CPU 부하를 줄이기 위해 30 FPS로 설정합니다.
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        try:
            profile = self.pipeline.start(config)
            self.get_logger().info("✅ 리얼센스 카메라 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ 카메라 시작 실패: {e}")
            return

        self.align = rs.align(rs.stream.color)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # 3. 좌표 발행 퍼블리셔 (마스터 노드가 구독하는 이름)
        self.target_pub = self.create_publisher(Point, '/aruco_target_point', 10)

        # 연산 주기를 0.03초(약 30 FPS)로 설정
        self.create_timer(0.03, self.process_frame)
        self.get_logger().info("🚀 6D Pose 스타일 비전 시스템 가동 시작")

    def process_frame(self):
        try:
            # 프레임 수신 및 정렬
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            display_img = color_image.copy()

            # 중앙 빨간 점 (화면 중심 기준점)
            cv2.circle(display_img, (320, 240), 4, (0, 0, 255), -1)

            # 🚀 [핵심 최적화] imgsz=320 설정으로 CPU 속도 향상
            results = self.model.predict(display_img, verbose=False, imgsz=320, device='cpu')[0]

            if results.boxes is not None:
                for box in results.boxes:
                    # 데이터 추출
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    conf = box.conf[0].cpu().numpy()
                    cls_id = int(box.cls[0])
                    cls_name = self.class_names[cls_id]
                    
                    # 객체 중심점(u, v) 계산 및 경계 처리
                    u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
                    u, v = max(0, min(u, 639)), max(0, min(v, 479))
                    
                    # 뎁스(거리) 값 추출
                    z_val = depth_frame.get_distance(u, v)

                    # 10cm ~ 1m 사이의 유효한 값만 처리
                    if 0.1 < z_val < 1.0: 
                        # 2D 픽셀 -> 3D 좌표 변환 (카메라 기준 좌표계)
                        point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
                        
                        
                        # 🌟 [좌표계 변환 핵심] 카메라 축을 로봇 베이스 축으로 매핑 (예시)
                        # 카메라 Z(깊이) -> 로봇 X(앞뒤)
                        # 카메라 X(가로) -> 로봇 -Y(좌우)
                        # 카메라 Y(세로) -> 로봇 -Z(상하)
                        robot_x = point_3d[0]
                        robot_y = point_3d[1]
                        robot_z = point_3d[2] - self.CAMERA_HEIGHT

                        # 4. 마스터 노드로 발행 (m 단위)
                        target_msg = Point()
                        target_msg.x = float(robot_x)
                        target_msg.y = float(robot_y)
                        target_msg.z = float(robot_z)
                        self.target_pub.publish(target_msg)

                        # 🎨 [시각화: 요청하신 스타일 적용]
                        blue_color = (255, 0, 0)
                        # 파란색 바운딩 박스
                        cv2.rectangle(display_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), blue_color, 2)
                        
                        # 상단 라벨 (이름 + 신뢰도)
                        label_top = f"{cls_name.upper()} {conf:.2f}"
                        (tw, th), _ = cv2.getTextSize(label_top, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        cv2.rectangle(display_img, (xyxy[0], xyxy[1] - th - 10), (xyxy[0] + tw, xyxy[1]), blue_color, -1)
                        cv2.putText(display_img, label_top, (xyxy[0], xyxy[1] - 7), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                        # 하단 XYZ 좌표 (mm 단위 표시)
                        label_bot = f"X:{robot_x*1000:.1f} Y:{robot_y*1000:.1f} Z:{robot_z*1000:.1f}"
                        cv2.putText(display_img, label_bot, (xyxy[0], xyxy[3] - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # FPS 표시
            curr_time = time.time()
            if self.prev_time > 0:
                fps = 1 / (curr_time - self.prev_time)
                cv2.putText(display_img, f"FPS: {fps:.1f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            self.prev_time = curr_time

            with self.frame_lock:
                self.latest_frame = display_img

        except Exception as e:
            pass

def main():
    rclpy.init()
    node = VisionNode()
    
    # GUI 출력을 위한 별도 스레드
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    try:
        while rclpy.ok():
            if node.latest_frame is not None:
                with node.frame_lock:
                    display_frame = node.latest_frame.copy()
                cv2.imshow("6D Pose Style Vision", display_frame)
            
            if cv2.waitKey(1) & 0xFF == 27: # ESC 누르면 종료
                break
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()