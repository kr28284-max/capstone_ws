import json
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32, String
from ultralytics import YOLO


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node3')

        # base_link 변환 파라미터 (관측 자세 기준)
        self.cam_origin_x_in_base = 0.023
        self.cam_origin_y_in_base = 0.000
        self.cam_origin_z_in_base = 0.26
        self.camera_to_gripper_x = -0.2
        self.camera_to_gripper_y = -0.0325

        # YOLO
        self.model = YOLO('/home/user/capstone_ws/best.pt')
        self.class_names = self.model.names
        self.prev_time = 0.0
        self.last_pub_log_time = 0.0

        # MediaPipe Hands
        mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_hands = mp_hands
        self.hands = mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        # RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        try:
            profile = self.pipeline.start(config)
            self.get_logger().info('Realsense camera started')
        except Exception as e:
            self.get_logger().error(f'Failed to start camera: {e}')
            raise

        self.align = rs.align(rs.stream.color)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # Publishers
        self.target_pub = self.create_publisher(Point, '/aruco_target_point', 10)
        self.detection_pub = self.create_publisher(String, '/vision/detections', 10)
        self.finger_pub = self.create_publisher(Int32, '/vision/hand_finger_count', 10)

        self.create_timer(0.03, self.process_frame)
        self.get_logger().info('vision_master_node3 started (YOLO + MediaPipe Hands)')

    @staticmethod
    def count_fingers(lm):
        fingers = [1 if lm[4][0] > lm[3][0] else 0]
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        for tip, pip in zip(tips, pips):
            fingers.append(1 if lm[tip][1] < lm[pip][1] else 0)
        return sum(fingers)

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not color_frame or not depth_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            display_img = color_image.copy()
            cv2.circle(display_img, (320, 240), 4, (0, 0, 255), -1)

            # Hand detection + finger count publish
            rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
            hand_result = self.hands.process(rgb)
            finger_count = -1
            if hand_result.multi_hand_landmarks:
                for hand_landmarks in hand_result.multi_hand_landmarks:
                    h, w, _ = display_img.shape
                    lm = [(int(p.x * w), int(p.y * h)) for p in hand_landmarks.landmark]
                    finger_count = self.count_fingers(lm)
                    self.mp_draw.draw_landmarks(display_img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    break

            finger_msg = Int32()
            finger_msg.data = int(finger_count)
            self.finger_pub.publish(finger_msg)
            if finger_count >= 0:
                cv2.putText(
                    display_img,
                    f'Fingers: {finger_count}',
                    (20, 85),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                )

            # YOLO detection
            results = self.model.predict(display_img, verbose=False, imgsz=320, device='cpu')[0]
            detections_payload = []

            if results.boxes is not None:
                for box in results.boxes:
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0])
                    cls_name = self.class_names[cls_id]

                    u = int((xyxy[0] + xyxy[2]) / 2)
                    v = int((xyxy[1] + xyxy[3]) / 2)
                    u = max(0, min(u, 639))
                    v = max(0, min(v, 479))

                    depth_m = depth_frame.get_distance(u, v)
                    if 0.10 < depth_m < 1.20:
                        point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)
                        cam_x = float(point_3d[0])
                        cam_y = float(point_3d[1])
                    else:
                        fx = self.intrinsics.fx
                        fy = self.intrinsics.fy
                        cx = self.intrinsics.ppx
                        cy = self.intrinsics.ppy
                        cam_x = (u - cx) / fx * self.cam_origin_z_in_base
                        cam_y = (v - cy) / fy * self.cam_origin_z_in_base

                    robot_x = self.cam_origin_x_in_base - cam_y
                    robot_y = self.cam_origin_y_in_base - cam_x

                    robot_x = robot_x - self.camera_to_gripper_x
                    robot_y = robot_y - self.camera_to_gripper_y
                    robot_z = 0.035

                    target_msg = Point()
                    target_msg.x = float(robot_x)
                    target_msg.y = float(robot_y)
                    target_msg.z = float(robot_z)
                    self.target_pub.publish(target_msg)

                    detections_payload.append(
                        {
                            'class_id': int(cls_id),
                            'class_name': str(cls_name).lower(),
                            'u': float(u),
                            'v': float(v),
                            'x': float(robot_x),
                            'y': float(robot_y),
                            'z': float(robot_z),
                        }
                    )

                    now = time.time()
                    if now - self.last_pub_log_time > 0.5:
                        self.get_logger().info(
                            f'publish /aruco_target_point (base_link): X={robot_x:.3f}, Y={robot_y:.3f}, Z={robot_z:.3f}'
                        )
                        self.last_pub_log_time = now

                    blue_color = (255, 0, 0)
                    cv2.rectangle(display_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), blue_color, 2)

                    label_top = f'{cls_name.upper()} {conf:.2f}'
                    (tw, th), _ = cv2.getTextSize(label_top, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(display_img, (xyxy[0], xyxy[1] - th - 10), (xyxy[0] + tw, xyxy[1]), blue_color, -1)
                    cv2.putText(
                        display_img,
                        label_top,
                        (xyxy[0], xyxy[1] - 7),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )

                    label_bot = f'X:{robot_x*1000:.1f} Y:{robot_y*1000:.1f} Z:{robot_z*1000:.1f}'
                    cv2.putText(
                        display_img,
                        label_bot,
                        (xyxy[0], xyxy[3] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

            detections_payload.sort(key=lambda d: d['u'])
            detections_msg = String()
            detections_msg.data = json.dumps({'detections': detections_payload}, ensure_ascii=False)
            self.detection_pub.publish(detections_msg)

            curr_time = time.time()
            if self.prev_time > 0:
                fps = 1 / (curr_time - self.prev_time)
                cv2.putText(display_img, f'FPS: {fps:.1f}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
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
                    display_frame = node.latest_frame.copy()
                cv2.imshow('Vision Master Node3 (YOLO + Hand)', display_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
