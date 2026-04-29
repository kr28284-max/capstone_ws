import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import threading
import pyrealsense2 as rs2

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        # 🌟 캡스톤용 YOLO 모델 경로 설정 (본인의 가중치 파일로 변경하세요)
        self.model = YOLO("/home/user/capstone_ws/boltnut.pt") 
        
        self.intrinsics = None
        self.latest_depth_image = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # 목표 좌표 발행 퍼블리셔 (마스터 노드와 통신)
        self.target_pub = self.create_publisher(Point, '/aruco_target_point', 10)

        # 구독자 설정 (네트워크를 통해 터틀봇 카메라 데이터 수신)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.img_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        self.get_logger().info("📷 캡스톤 비전 노드 시작: YOLO 객체 인식 및 XYZ/Depth 추출 중...")

    def info_callback(self, msg):
        # ROS CameraInfo를 RealSense intrinsics 객체로 변환 (3D 좌표 계산용)
        if self.intrinsics is None:
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs2.distortion.brown_conrady
            self.intrinsics.coeffs = list(msg.d)

    def depth_callback(self, msg):
        # 16비트 Depth 이미지를 numpy 배열로 저장 (단위: mm)
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def image_callback(self, msg):
        if self.intrinsics is None or self.latest_depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            display_img = cv_image.copy()
            
            # 화면 중심점 표시
            cv2.circle(display_img, (320, 240), 5, (255, 255, 255), -1)

            # YOLO 추론
            results = self.model(display_img, verbose=False)[0]

            if results.boxes is not None:
                for box in results.boxes:
                    # 바운딩 박스 좌표 및 중심점 계산
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
                    
                    # 중심점의 Depth 값 추출 (mm -> m 변환)
                    depth_mm = self.latest_depth_image[v, u]
                    z_val = depth_mm / 1000.0

                    if z_val > 0:
                        # 픽셀 좌표와 Depth를 이용해 실제 3D 좌표(X, Y, Z) 계산
                        point_3d = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
                        x_r, y_r, z_r = point_3d[0], point_3d[1], point_3d[2]

                        # 마스터 노드로 목표물 좌표 퍼블리시
                        target_msg = Point()
                        target_msg.x = x_r
                        target_msg.y = y_r
                        target_msg.z = z_r
                        self.target_pub.publish(target_msg)

                        # 화면에 박스 및 XYZ, Depth 정보 텍스트 표시
                        cv2.rectangle(display_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                        text = f"X:{x_r*1000:.0f} Y:{y_r*1000:.0f} Z:{z_r*1000:.0f} D:{z_val:.3f}m"
                        cv2.putText(display_img, text, (xyxy[0], xyxy[1] - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 메인 스레드 출력을 위해 프레임 저장
            with self.frame_lock:
                self.latest_frame = display_img

        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

def main():
    rclpy.init()
    node = VisionNode()
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    try:
        while rclpy.ok():
            if node.latest_frame is not None:
                with node.frame_lock:
                    display_frame = node.latest_frame.copy()
                cv2.imshow("Capstone Vision - YOLO XYZ/Depth", display_frame)
            
            if cv2.waitKey(30) & 0xFF == 27: # ESC 종료
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()