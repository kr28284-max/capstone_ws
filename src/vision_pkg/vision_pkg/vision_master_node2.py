import json
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
import rclpy
from collections import deque, Counter
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from ultralytics import YOLO


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_master_node2')

        # base_link 변환 파라미터 (관측 자세 기준)
        self.cam_origin_x_in_base = 0.19
        self.cam_origin_y_in_base = 0.0225
        self.cam_origin_z_in_base = 0.177
        self.camera_to_gripper_x = -0.055
        self.camera_to_gripper_y = -0.0225
        self.camera_to_gripper_z = -0.072
        self.depth_min_m = 0.10
        self.depth_max_m = 1.20

        # YOLO
        self.model = YOLO('/home/user2/capstone_ws/best.pt')
        self.class_names = self.model.names
        self.class_aliases = {
            'bearing': 'bearing',
            'bolt_nut': 'boltnut',
            'bolt-nut': 'boltnut',
            'bolt nut': 'boltnut',
            'boltnut': 'boltnut',
            'gear': 'gear',
            'damper': 'damper',
        }
        self.prev_time = 0.0
        self.last_pub_log_time = 0.0
        self.last_frame_timeout_log_time = 0.0
        self.yolo_enabled = True


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
        self.hand_detected_pub = self.create_publisher(Bool, '/vision/hand_detected', 10)

        self.mode = 'HAND'
        self.class_map = {
            1: 'bearing',
            2: 'boltnut',
            3: 'gear',
            4: 'damper',
        }
        self.scan_history = deque(maxlen=30)
        self.scan_start_time = None
        self.SCAN_DURATION = 2.0
        self.object_start_time = None
        self.OBJECT_DURATION = 4.0
        self.mediapipe_pause_until = 0
        self.HAND_CLOSE_MIN_SIZE = 120
        self.hand_return_start = None
        self.HAND_RETURN_DURATION = 1.5
        self.selected_class = None
        self.last_results = None

        # Subscribers
        self.yolo_enable_sub = self.create_subscription(Bool, '/vision/yolo_enable', self.yolo_enable_callback, 10)

        self.create_timer(0.03, self.process_frame)
        self.get_logger().info('vision_master_node2 started (YOLO + MediaPipe Hands)')

    def yolo_enable_callback(self, msg: Bool):
        self.yolo_enabled = True
        self.get_logger().info('YOLO always enabled')

    def get_valid_depth(self, depth_frame, u: int, v: int):
        depths = []
        for dv in range(-3, 4):
            py = max(0, min(v + dv, 479))
            for du in range(-3, 4):
                px = max(0, min(u + du, 639))
                depth_m = depth_frame.get_distance(px, py)
                if self.depth_min_m < depth_m < self.depth_max_m:
                    depths.append(depth_m)
        if not depths:
            return None
        return float(np.median(depths))

    @staticmethod
    def count_fingers(lm, hand_label):
        fingers = []
        if hand_label == 'Right':
            fingers.append(1 if lm[4][0] < lm[3][0] else 0)
        else:
            fingers.append(1 if lm[4][0] > lm[3][0] else 0)

        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        for tip, pip in zip(tips, pips):
            fingers.append(1 if lm[tip][1] < lm[pip][1] else 0)
        return sum(fingers)

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not color_frame or not depth_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            display_img = color_image.copy()
            h, w, _ = display_img.shape
            cv2.circle(display_img, (320, 240), 4, (0, 0, 255), -1)
            current_time = time.time()

            def put_text_right(text, y, font_scale=0.7, color=(255, 255, 255), thickness=2):
                (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
                x = max(10, w - tw - 10)
                cv2.putText(display_img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

            # Hand detection + finger count publish
            rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
            hand_result = self.hands.process(rgb)
            finger_count = -1
            hand_detected = False
            hand_close = False
            if hand_result.multi_hand_landmarks:
                for idx, hand_landmarks in enumerate(hand_result.multi_hand_landmarks):
                    h, w, _ = display_img.shape
                    lm = [(int(p.x * w), int(p.y * h)) for p in hand_landmarks.landmark]

                    xs = [pt[0] for pt in lm]
                    ys = [pt[1] for pt in lm]
                    hand_size = max(max(xs) - min(xs), max(ys) - min(ys))
                    if hand_size >= self.HAND_CLOSE_MIN_SIZE:
                        hand_label = (
                            hand_result.multi_handedness[idx]
                            .classification[0]
                            .label
                        )
                        finger_count = self.count_fingers(lm, hand_label)
                        hand_detected = True
                        hand_close = True

                        self.mp_draw.draw_landmarks(display_img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                        display_label = 'Left' if hand_label == 'Right' else 'Right'
                        put_text_right(
                            f"{display_label} Hand",
                            90,
                            font_scale=0.6,
                            color=(255, 255, 0),
                            thickness=2,
                        )
                    else:
                        cv2.putText(
                            display_img,
                            "HAND TOO FAR",
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 0, 255),
                            2
                        )
                    break

            finger_msg = Int32()
            finger_msg.data = int(finger_count)
            self.finger_pub.publish(finger_msg)
            hand_msg = Bool()
            hand_msg.data = bool(hand_detected)
            self.hand_detected_pub.publish(hand_msg)

            if self.mode == 'HAND':
                if finger_count in self.class_map and hand_close:
                    if self.scan_start_time is None:
                        self.scan_start_time = current_time
                    self.scan_history.append(finger_count)
                    if current_time - self.scan_start_time > self.SCAN_DURATION:
                        most_common = Counter(self.scan_history).most_common(1)[0][0]
                        self.selected_class = self.class_map[most_common]
                        self.mode = 'OBJECT'
                        self.object_start_time = current_time
                        self.mediapipe_pause_until = current_time + 2.0
                        self.scan_history.clear()
                        self.scan_start_time = None
                else:
                    self.scan_start_time = None
                    self.scan_history.clear()

                if finger_count >= 0:
                    put_text_right(
                        f'Fingers: {finger_count}',
                        115,
                        font_scale=0.8,
                        color=(0, 255, 255),
                        thickness=2,
                    )
                if self.scan_start_time:
                    put_text_right(
                        'SCANNING...',
                        145,
                        font_scale=0.6,
                        color=(0, 255, 255),
                        thickness=2,
                    )
            else:
                if hand_close:
                    if self.hand_return_start is None:
                        self.hand_return_start = current_time
                    elif current_time - self.hand_return_start > self.HAND_RETURN_DURATION:
                        self.mode = 'HAND'
                        self.selected_class = None
                        self.last_results = None
                        self.scan_history.clear()
                        self.scan_start_time = None
                        self.hand_return_start = None

                        put_text_right(
                            'RETURN TO HAND MODE',
                            120,
                            font_scale=0.7,
                            color=(0, 0, 255),
                            thickness=2,
                        )
                else:
                    self.hand_return_start = None

                if self.selected_class:
                    put_text_right(
                        f'TARGET: {self.selected_class}',
                        30,
                        font_scale=0.7,
                        color=(255, 0, 0),
                        thickness=2,
                    )
                if current_time < self.mediapipe_pause_until:
                    put_text_right(
                        'Waiting',
                        60,
                        font_scale=0.6,
                        color=(0, 165, 255),
                        thickness=2,
                    )
                else:
                    put_text_right(
                        'SHOW HAND TO RETURN',
                        60,
                        font_scale=0.6,
                        color=(0, 255, 255),
                        thickness=2,
                    )

                if self.object_start_time is not None and current_time - self.object_start_time > self.OBJECT_DURATION:
                    self.mode = 'HAND'
                    self.selected_class = None
                    self.last_results = None
                    self.scan_history.clear()
                    self.scan_start_time = None
                    self.hand_return_start = None

            detections_payload = []
            if self.yolo_enabled:
                results = self.model.predict(display_img, verbose=False, imgsz=320, device='cpu')[0]
                self.last_results = results

                if results.boxes is not None:
                    for box in results.boxes:
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        conf = float(box.conf[0].cpu().numpy())
                        cls_id = int(box.cls[0])
                        cls_name = self.class_names[cls_id]
                        cls_name_lower = str(cls_name).lower()
                        mapped_name = self.class_aliases.get(cls_name_lower, cls_name_lower)

                        u = int((xyxy[0] + xyxy[2]) / 2)
                        v = int((xyxy[1] + xyxy[3]) / 2)
                        u = max(0, min(u, 639))
                        v = max(0, min(v, 479))

                        depth_m = self.get_valid_depth(depth_frame, u, v)
                        if depth_m is None:
                            continue

                        point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)
                        cam_x = float(point_3d[0])
                        cam_y = float(point_3d[1])
                        cam_z = float(point_3d[2])

                        robot_x = self.cam_origin_x_in_base - cam_y
                        robot_y = self.cam_origin_y_in_base - cam_x
                        robot_z = self.cam_origin_z_in_base - cam_z 

                        robot_x = robot_x - self.camera_to_gripper_x
                        robot_y = robot_y - self.camera_to_gripper_y
                        robot_z = robot_z - self.camera_to_gripper_z

                        if self.mode == 'OBJECT' and self.selected_class and mapped_name == self.selected_class:
                            target_msg = Point()
                            target_msg.x = float(robot_x)
                            target_msg.y = float(robot_y)
                            target_msg.z = float(robot_z)
                            self.target_pub.publish(target_msg)

                            now = time.time()
                            if now - self.last_pub_log_time > 0.5:
                                self.get_logger().info(
                                    f'publish /aruco_target_point (base_link): X={robot_x:.3f}, Y={robot_y:.3f}, Z={robot_z:.3f}'
                                )
                                self.last_pub_log_time = now

                        detections_payload.append(
                            {
                                'class_id': int(cls_id),
                                'class_name': mapped_name,
                                'u': float(u),
                                'v': float(v),
                                'x': float(robot_x),
                                'y': float(robot_y),
                                'z': float(robot_z),
                            }
                        )

                        blue_color = (255, 0, 0)
                        cv2.rectangle(display_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), blue_color, 2)

                        # 표시 라벨은 매핑된 이름 사용
                        label_top = f'{mapped_name.upper()} {conf:.2f}'
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

        except Exception as e:
            if 'Frame didn\'t arrive' in str(e):
                now = time.time()
                if now - self.last_frame_timeout_log_time > 2.0:
                    self.get_logger().warn(f'RealSense frame timeout: {e}')
                    self.last_frame_timeout_log_time = now
                return
            self.get_logger().error(f'process_frame error: {e}')


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
                cv2.imshow('Vision Master Node2 (YOLO + Hand)', display_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
