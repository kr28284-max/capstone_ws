import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        # 고정 기준 좌표 설정
        self.base_target_x = 0.134
        self.base_target_y = 0.0
        
        # 아루코 설정
        self.marker_length = 0.024
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.intrinsics = None
        self.dist_coeffs = None

        # 퍼블리셔 (목표 좌표 발행)
        self.target_pub = self.create_publisher(Point, '/aruco_target_point', 10)

        # 구독자
        self.img_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)

        self.get_logger().info("📷 비전 노드 시작: 아루코 마커 감지 중...")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            rx, ry, rz = 0.0, 0.0, 0.0
            marker_detected = False

            if self.intrinsics is not None:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.detector.detectMarkers(gray)

                if ids is not None:
                    obj_points = np.array([
                        [-self.marker_length/2, self.marker_length/2, 0],
                        [ self.marker_length/2, self.marker_length/2, 0],
                        [ self.marker_length/2, -self.marker_length/2, 0],
                        [-self.marker_length/2, -self.marker_length/2, 0]
                    ], dtype=np.float32)

                    _, rvec, tvec = cv2.solvePnP(obj_points, corners[0], self.intrinsics, self.dist_coeffs)
                    cx, cy, cz = tvec[0][0], tvec[1][0], tvec[2][0]

                    # 카메라 좌표 -> 로봇 좌표 변환
                    rx = self.base_target_x - cy + 0.155
                    ry = self.base_target_y - cx + 0.058
                    rz = 0.175
                    
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                    marker_detected = True
                    
                    # 거리가 적절할 때만 좌표 발행
                    if 0.15 <= cz <= 0.45:
                        target_msg = Point()
                        target_msg.x = rx
                        target_msg.y = ry
                        target_msg.z = rz
                        self.target_pub.publish(target_msg)

                # HUD 표시
                font = cv2.FONT_HERSHEY_SIMPLEX
                color = (0, 255, 0) if marker_detected else (0, 0, 255)
                cv2.rectangle(cv_image, (10, 10), (360, 160), (0,0,0), -1)
                cv2.addWeighted(cv_image, 0.6, cv_image, 0.4, 0, cv_image)
                cv2.putText(cv_image, f"Target X: {rx:.3f}m", (20, 50), font, 0.8, color, 2)
                cv2.putText(cv_image, f"Target Y: {ry:.3f}m", (20, 95), font, 0.8, color, 2)
                cv2.putText(cv_image, f"Target Z: {rz:.3f}m", (20, 140), font, 0.8, color, 2)

            cv2.imshow("Vision Node - ArUco", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()