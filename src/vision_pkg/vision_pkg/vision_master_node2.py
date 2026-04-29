# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# import pyrealsense2 as rs
# from ultralytics import YOLO
# import cv2
# import numpy as np
# import threading
# import time

# class VisionNode(Node):
#     def __init__(self):
#         super().__init__('vision_node')

#         # [수정] CPU 환경에 맞춰 설정
#         self.get_logger().info("💻 그래픽카드가 없어 CPU 모드로 동작합니다.")

#         # [수정] 모델은 한 번만 로드합니다. (중복 선언 제거)
#         self.model = YOLO("/home/user/capstone_ws/wheel.pt")
#         self.class_names = self.model.names
        
#         # [수정] FPS 계산을 위한 변수 초기화 (누락되었던 부분)
#         self.prev_time = 0
        
#         # 2. 리얼센스 파이프라인 설정
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         # CPU 부하를 줄이기 위해 60 FPS가 너무 무거우면 30으로 낮추는 것이 좋습니다.
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
#         profile = self.pipeline.start(config)
#         self.align = rs.align(rs.stream.color)
#         self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

#         self.latest_frame = None
#         self.frame_lock = threading.Lock()

#         # 목표 좌표 발행 퍼블리셔
#         self.target_pub = self.create_publisher(Point, '/aruco_target_point', 10)

#         # 타이머 주기 (0.01초는 CPU에 무리가 갈 수 있어 0.03(30fps) 정도로 조정 추천)
#         self.create_timer(0.03, self.process_frame)
        
#         self.get_logger().info("🚀 로보컵 스타일 비전 노드 시작 (CPU 모드)")

#     def process_frame(self):
#         try:
#             frames = self.pipeline.wait_for_frames(timeout_ms=100)
#             aligned_frames = self.align.process(frames)
#             depth_frame = aligned_frames.get_depth_frame()
#             color_frame = aligned_frames.get_color_frame()

#             if not depth_frame or not color_frame:
#                 return

#             color_image = np.asanyarray(color_frame.get_data())
#             display_img = color_image.copy()

#             # [수정] half=True는 GPU 전용 옵션입니다. CPU에서는 빼거나 False로 해야 에러가 안 납니다.
#             results = self.model.predict(display_img, verbose=False, device='cpu')[0]

#             if results.boxes is not None:
#                 for box in results.boxes:
#                     xyxy = box.xyxy[0].cpu().numpy().astype(int)
#                     cls_id = int(box.cls[0])
#                     cls_name = self.class_names[cls_id]
                    
#                     u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
#                     u, v = max(0, min(u, 639)), max(0, min(v, 479))
                    
#                     z_val = depth_frame.get_distance(u, v)

#                     if 0.1 < z_val < 1.0: 
#                         point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
#                         x_r, y_r, z_r = point_3d[0], point_3d[1], point_3d[2]

#                         target_msg = Point()
#                         target_msg.x = float(x_r)
#                         target_msg.y = float(y_r)
#                         target_msg.z = float(z_r)
#                         self.target_pub.publish(target_msg)

#                         # 시각화 부분
#                         color = (0, 255, 0)
#                         cv2.rectangle(display_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)
                        
#                         label = f"{cls_name.upper()} | {z_val*1000:.0f}mm"
#                         (label_w, label_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
#                         cv2.rectangle(display_img, (xyxy[0], xyxy[1]-label_h-10), (xyxy[0]+label_w, xyxy[1]), (0,0,0), -1)
#                         cv2.putText(display_img, label, (xyxy[0], xyxy[1]-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

#             # FPS 표시
#             curr_time = time.time()
#             if self.prev_time > 0:
#                 fps = 1 / (curr_time - self.prev_time)
#                 cv2.putText(display_img, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
#             self.prev_time = curr_time

#             with self.frame_lock:
#                 self.latest_frame = display_img

#         except Exception as e:
#             self.get_logger().error(f"Error: {e}")

# def main():
#     rclpy.init()
#     node = VisionNode()
#     ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     ros_thread.start()
    
#     try:
#         while rclpy.ok():
#             if node.latest_frame is not None:
#                 with node.frame_lock:
#                     cv2.imshow("Robocup Style Vision", node.latest_frame)
#             if cv2.waitKey(1) & 0xFF == 27:
#                 break
#     finally:
#         node.pipeline.stop()
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
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

        # [최적화] CPU 환경 최적화 설정
        self.model = YOLO("/home/user/capstone_ws/best.pt")
        self.class_names = self.model.names
        self.prev_time = 0
        
        # 리얼센스 파이프라인 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.target_pub = self.create_publisher(Point, '/aruco_target_point', 10)

        # 타이머 설정
        self.create_timer(0.01, self.process_frame)
        self.get_logger().info("🚀 6D Pose 스타일 시각화 모드 가동")

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame: return

            color_image = np.asanyarray(color_frame.get_data())
            display_img = color_image.copy()

            # 중앙 빨간 점 (화면 중심 기준점)
            cv2.circle(display_img, (320, 240), 4, (0, 0, 255), -1)

            # YOLO 추론
            results = self.model.predict(display_img, verbose=False, imgsz=320, device='cpu')[0]

            if results.boxes is not None:
                for box in results.boxes:
                    # 데이터 추출
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    conf = box.conf[0].cpu().numpy()
                    cls_id = int(box.cls[0])
                    cls_name = self.class_names[cls_id]
                    
                    u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
                    u, v = max(0, min(u, 639)), max(0, min(v, 479))
                    z_val = depth_frame.get_distance(u, v)

                    if 0.1 < z_val < 1.0: 
                        point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
                        # 사진처럼 mm 단위로 표시하려면 *1000을 해줍니다.
                        rx, ry, rz = point_3d[0]*1000, point_3d[1]*1000, point_3d[2]*1000

                        target_msg = Point()
                        target_msg.x, target_msg.y, target_msg.z = float(rx), float(ry), float(rz)
                        self.target_pub.publish(target_msg)

                        # 🎨 [시각화: 사진과 동일한 스타일 적용]
                        # 1. 파란색 박스 (BGR: 255, 0, 0)
                        blue_color = (255, 0, 0)
                        cv2.rectangle(display_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), blue_color, 2)
                        
                        # 2. 상단 클래스 라벨 (파란색 배경 + 흰색 글씨)
                        label_top = f"{cls_name} {conf:.2f}"
                        (tw, th), _ = cv2.getTextSize(label_top, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        cv2.rectangle(display_img, (xyxy[0], xyxy[1] - th - 10), (xyxy[0] + tw, xyxy[1]), blue_color, -1)
                        cv2.putText(display_img, label_top, (xyxy[0], xyxy[1] - 7), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                        # 3. 하단 XYZ 좌표 (밝은 초록색 글씨, BGR: 0, 255, 0)
                        label_bot = f"X:{rx:.1f} Y:{ry:.1f} Z:{rz:.1f}"
                        # 박스 내부 하단에 위치하도록 조정
                        cv2.putText(display_img, label_bot, (xyxy[0], xyxy[3] - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # FPS 표시
            curr_time = time.time()
            if self.prev_time > 0:
                fps = 1 / (curr_time - self.prev_time)
                cv2.putText(display_img, f"FPS: {fps:.1f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            self.prev_time = curr_time

            with self.frame_lock:
                self.latest_frame = display_img

        except Exception:
            pass

def main():
    rclpy.init()
    node = VisionNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    try:
        while rclpy.ok():
            if node.latest_frame is not None:
                with node.frame_lock:
                    cv2.imshow("6D Pose (Refined Style)", node.latest_frame)
            if cv2.waitKey(1) & 0xFF == 27: break
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()