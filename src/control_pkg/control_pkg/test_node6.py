import json
import math
import threading
import time
from typing import Dict, List, Optional

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestNode6(Node):
    def __init__(self):
        super().__init__('capstone_final')

        self.callback_group = ReentrantCallbackGroup()
        self.ready = False
        self.busy = True

        self.goal_frame = 'base_link'
        self.ee_link_name = 'end_effector_link'

        self.recognition_wait_sec = 4.0
        self.detect_timeout_sec = 2.0
        self.class_check_timeout_sec = 2.0
        self.joint_motion_wait_sec = 2.5
        self.fast_transfer_motion_sec = 1.7
        self.place_transfer_motion_sec = 1.8

        self.pick_z_min = 0.0
        self.pick_z_max = 20.0
        # self.far_x_threshold_m = 0.230
        self.near_after_pick_lift_height = 0.22
        self.after_pick_waypoint_joints = [0.0, -32.0, -15.0, 100.0]
        # self.place_hover_joints = {
        #     1: [107.0, -20.0, 15.0, 88.0],
        #     2: [152.0, 5.0, -20.0, 77.0],
        #     3: [-149.0, 5.0, -20.0, 77.0],
        #     4: [-107.0, -20.0, 15.0, 88.0],
        # }
        self.place_hover_joints = {
            1: [118.0, -19.0, 14.0, 88.0],
            2: [157.0, 12.0, -26.0, 73.0],
            3: [-157.0, 12.0, -26.0, 73.0],
            4: [-118.0, -19.0, 14.0, 88.0],
        }

        self.vision_joint_deg = [0.0, -6.0, -24.0, 114.0]
        self.home_joint_deg = [0.0, -6.0, -24.0, 114.0]
        self.hand_recognition_joint_deg = [0.0, -60.0, 20.0, 10.0]
        # Pick offsets by command: 1=wheel, 2=boltnut, 3=gear, 4=bearing.
        #bearing
        self.bearing_pick_offset_x = -0.008
        self.bearing_pick_offset_y = 0.0
        self.bearing_pick_offset_z = 0.053
        #boltnut
        self.boltnut_pick_offset_x = -0.018
        self.boltnut_pick_offset_y = 0.0
        self.boltnut_pick_offset_z = 0.058
        #gear
        self.gear_pick_offset_x = -0.005
        self.gear_pick_offset_y = 0.0
        self.gear_pick_offset_z = 0.038
        #wheel
        self.wheel_pick_offset_x = 0.0
        self.wheel_pick_offset_y = 0.0
        self.wheel_pick_offset_z = 0.040


        self.near_x_threshold_m = 0.210
        self.near_x_threshold_h = 0.240

#########################
        self.bearing_near_pick_offset_z = 0.02

        self.boltnut_near_pick_offset_z = -0.02

        self.gear_near_pick_offset_z = 0.015

        self.wheel_near_pick_offset_z = 0.02

#########################
        self.near_bearing_wheel_x_threshold_m = 0.233
        self.bearing_near_pick_offset_x = -0.015

        self.wheel_near_pick_offset_x = -0.015

#########################
        self.near_bearing_wheel_low_x_threshold_m = 0.205

        self.bearing_near_low_x_pick_offset_z = 0.03

        self.wheel_near_low_x_pick_offset_z = 0.0

        self.boltnut_near_low_x_pick_offset_x = -0.01

        self.near_gear_x_threshold_m = 0.230

        self.gear_near_pick_offset_x = -0.015
        self.gear_near_high_y_threshold_m = 0.085
        self.gear_near_high_y_pick_offset_x = -0.02

        self.low_y_threshold_m = -0.030

#########################
        self.bearing_low_y_pick_offset_y = -0.02

        self.boltnut_low_y_pick_offset_y = -0.02

        self.gear_low_y_pick_offset_y = -0.015

        self.wheel_low_y_pick_offset_y = -0.02

#########################
        self.high_y_threshold_m = 0.100

#########################
        self.bearing_high_y_pick_offset_y = 0.02

        self.boltnut_high_y_pick_offset_y = 0.02

        self.gear_high_y_pick_offset_y = 0.02

        self.wheel_high_y_pick_offset_y = 0.02

        self.pick_offsets = {
            1: (self.wheel_pick_offset_x, self.wheel_pick_offset_y, self.wheel_pick_offset_z),
            2: (self.boltnut_pick_offset_x, self.boltnut_pick_offset_y, self.boltnut_pick_offset_z),
            3: (self.gear_pick_offset_x, self.gear_pick_offset_y, self.gear_pick_offset_z),
            4: (self.bearing_pick_offset_x, self.bearing_pick_offset_y, self.bearing_pick_offset_z),
        }
        self.near_pick_offset_z = {
            1: self.wheel_near_pick_offset_z,
            2: self.boltnut_near_pick_offset_z,
            3: self.gear_near_pick_offset_z,
            4: self.bearing_near_pick_offset_z,
        }
        self.low_y_pick_offset_y = {
            1: self.wheel_low_y_pick_offset_y,
            2: self.boltnut_low_y_pick_offset_y,
            3: self.gear_low_y_pick_offset_y,
            4: self.bearing_low_y_pick_offset_y,
        }
        self.high_y_pick_offset_y = {
            1: self.wheel_high_y_pick_offset_y,
            2: self.boltnut_high_y_pick_offset_y,
            3: self.gear_high_y_pick_offset_y,
            4: self.bearing_high_y_pick_offset_y,
        }

        self.command_name = {
            1: 'wheel',
            2: 'boltnut',
            3: 'gear',
            4: 'bearing',
        }
        self.command_class_id = {
            1: 3,
            2: 1,
            3: 2,
            4: 0,
        }

        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.arm_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.callback_group)
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group,
        )

        self.detection_sub = self.create_subscription(
            String,
            '/vision/detections',
            self.detection_callback,
            10,
            callback_group=self.callback_group,
        )
        self.detection_sub_alt = self.create_subscription(
            String,
            '/vision/detections5',
            self.detection_callback,
            10,
            callback_group=self.callback_group,
        )
        self.command_sub = self.create_subscription(
            Int32,
            '/pick_command',
            self.command_callback,
            10,
            callback_group=self.callback_group,
        )
        self.hand_detected_sub = self.create_subscription(
            Bool,
            '/vision/hand_detected',
            self.hand_detected_callback,
            10,
            callback_group=self.callback_group,
        )
        self.hand_sub = self.create_subscription(
            Int32,
            '/vision/hand_finger_count',
            self.hand_finger_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.latest_detections: List[Dict] = []
        self.latest_detections_stamp = 0.0
        self.det_lock = threading.Lock()
        self.current_joint_positions: Optional[List[float]] = None
        self.current_joint_lock = threading.Lock()
        self.last_finger = -1
        self.same_finger_count = 0
        self.required_stable_count = 12
        self.last_hand_trigger_time = 0.0
        self.hand_trigger_cooldown_sec = 2.0
        self.hand_pause_event = threading.Event()
        self.hand_pause_event.set()
        self.hand_pause_lock = threading.Lock()
        self.hand_pause_active = False
        self.hand_pause_monitor_enabled = False
        self.pause_input_active = False

        self.get_logger().info('START test_node6')
        threading.Thread(target=self.init_robot_sequence, daemon=True).start()

    def init_robot_sequence(self):
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_action 서버 연결 실패')
            return
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('gripper_cmd 서버 연결 실패')
            return

        self.send_arm_joint_topic(self.hand_recognition_joint_deg)
        self.wait_joint_motion('손가락 인식자세 이동')
        self.send_gripper_blocking(0.019)

        self.ready = True
        self.busy = False
        self.get_logger().info('READY 손가락 입력 대기 (/vision/hand_finger_count: 1~4)')

    def detection_callback(self, msg: String):
        try:
            parsed = json.loads(msg.data)
            dets = parsed.get('detections', [])
            if not isinstance(dets, list):
                return

            normalized = []
            for d in dets:
                if not isinstance(d, dict):
                    continue
                normalized.append(
                    {
                        'class_id': int(d.get('class_id', -1)),
                        'class_name': str(d.get('class_name', '')).lower(),
                        'u': float(d.get('u', 1e9)),
                        'x': float(d.get('x', 0.0)),
                        'y': float(d.get('y', 0.0)),
                        'z': float(d.get('z', 0.0)),
                    }
                )

            with self.det_lock:
                self.latest_detections = normalized
                self.latest_detections_stamp = time.time()
        except Exception:
            return

    def command_callback(self, msg: Int32):
        self.request_command(int(msg.data), source='topic')

    def joint_state_callback(self, msg: JointState):
        if not msg.position:
            return

        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        positions_by_name = dict(zip(msg.name, msg.position))
        if all(name in positions_by_name for name in joint_names):
            positions = [float(positions_by_name[name]) for name in joint_names]
        elif len(msg.position) >= 4:
            positions = [float(pos) for pos in msg.position[:4]]
        else:
            return

        with self.current_joint_lock:
            self.current_joint_positions = positions

    def hand_detected_callback(self, msg: Bool):
        if msg.data:
            self.request_hand_safety_pause()

    def clear_latest_detections(self):
        with self.det_lock:
            self.latest_detections = []
            self.latest_detections_stamp = 0.0

    def hand_finger_callback(self, msg: Int32):
        finger = int(msg.data)

        
        if finger < 1 or finger > 4:
            self.last_finger = finger
            self.same_finger_count = 0
            return

        if finger == self.last_finger:
            self.same_finger_count += 1
        else:
            self.last_finger = finger
            self.same_finger_count = 1

        if self.same_finger_count < self.required_stable_count:
            return

        now = time.time()
        if now - self.last_hand_trigger_time < self.hand_trigger_cooldown_sec:
            return

        # ?��?�� ?���??�� ?���? ?��?��?��?�� 중복 발사�? 막기 ?��?�� 콜백 카운?�� 리셋
        self.same_finger_count = 0
        self.last_hand_trigger_time = now
        self.request_command(finger, source='hand')

    def request_command(self, cmd: int, source: str):
        if cmd not in self.command_name:
            self.get_logger().warn(f'[{source}] 지원하지 않는 명령: {cmd}')
            return
        if not self.ready:
            self.get_logger().warn(f'[{source}] 아직 초기화 중')
            return
        if self.busy:
            self.get_logger().warn(f'[{source}] 현재 작업 중')
            return

        self.busy = True
        self.get_logger().info(f'[{source}] 명령 접수: {cmd}({self.command_name[cmd]})')
        threading.Thread(target=self.run_command_sequence, args=(cmd,), daemon=True).start()

    def request_hand_safety_pause(self):
        with self.hand_pause_lock:
            if not self.busy or not self.hand_pause_monitor_enabled or self.hand_pause_active:
                return
            self.hand_pause_active = True
            self.hand_pause_event.clear()
            should_start_input_thread = not self.pause_input_active
            self.pause_input_active = True

        self.get_logger().warn('작업 중 손 감지: 로봇팔 정지 후 okay 입력 대기')
        self.stop_arm_motion()
        self.clear_latest_detections()

        if should_start_input_thread:
            threading.Thread(target=self.wait_for_input, daemon=True).start()

    def wait_for_input(self):
        while rclpy.ok():
            try:
                text = input('손을 치운 뒤 okay 입력 > ').strip().lower()
            except EOFError:
                self.get_logger().error('터미널 입력을 사용할 수 없어 일시정지를 해제할 수 없습니다.')
                time.sleep(1.0)
                continue
            except Exception as exc:
                self.get_logger().warn(f'okay 입력 오류: {exc}')
                time.sleep(0.2)
                continue

            if text == 'okay':
                with self.hand_pause_lock:
                    self.hand_pause_active = False
                    self.pause_input_active = False
                    self.hand_pause_event.set()
                self.clear_latest_detections()
                self.get_logger().info('okay 입력 확인. 중단된 작업을 다시 진행합니다.')
                return

            self.get_logger().warn('계속하려면 okay 를 입력하세요.')

    def wait_if_hand_paused(self):
        was_paused = False
        while rclpy.ok() and not self.hand_pause_event.wait(0.1):
            was_paused = True
        return was_paused

    def run_command_sequence(self, cmd: int):
        name = self.command_name[cmd]
        count = 0
        self.get_logger().info(f'{cmd}({name})')
        try:
            while rclpy.ok():
                self.hand_pause_monitor_enabled = False
                self.send_arm_joint_topic_blocking(self.vision_joint_deg, '물체 인식자세 이동')
                self.hand_pause_monitor_enabled = True
                self.wait_if_hand_paused()
                self.clear_latest_detections()

                first_target = self.find_leftmost_target(cmd, self.class_check_timeout_sec)
                self.wait_if_hand_paused()
                if first_target is None:
                    self.get_logger().info(f'{name} 추가 물체 없음. 총 {count}개 처리 후 종료')
                    break

                self.get_logger().info(f'{name} 좌표 안정화를 위해 대기')
                self.wait_recognition()
                self.wait_if_hand_paused()

                target = self.find_leftmost_target(cmd, self.detect_timeout_sec)
                self.wait_if_hand_paused()
                if target is None:
                    self.get_logger().info(f'{name} class 확인 후 좌표 확정 실패. 총 {count}개 처리 후 종료')
                    break
                okay = self.execute_enhanced_sequence(cmd, target)
                if not okay:
                    self.get_logger().warn('시퀀스 실패로 중단')
                    break
                count += 1
        finally:
            self.hand_pause_monitor_enabled = False
            self.send_arm_joint_topic_blocking(self.hand_recognition_joint_deg, '손가락 인식자세 복귀')
            self.busy = False
            self.get_logger().info('FINISH 다음 명령 대기')

    def find_leftmost_target(self, cmd: int, timeout_sec: float) -> Optional[Dict]:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if not self.hand_pause_event.is_set():
                self.wait_if_hand_paused()
                deadline = time.time() + timeout_sec

            with self.det_lock:
                dets = list(self.latest_detections)
                stamp = self.latest_detections_stamp

            if dets and (time.time() - stamp) < 2.0:
                candidates = [d for d in dets if self.is_command_match(cmd, d)]
                if candidates:
                    candidates.sort(key=lambda d: d['u'])
                    return candidates[0]
            time.sleep(0.05)
        return None

    def is_command_match(self, cmd: int, det: Dict) -> bool:
        if det.get('class_name', '') == self.command_name[cmd]:
            return True
        return int(det.get('class_id', -1)) == self.command_class_id[cmd]

    def execute_enhanced_sequence(self, cmd: int, target: Dict) -> bool:
        raw_tx = float(target['x'])
        raw_ty = float(target['y'])
        base_x_offset, base_y_offset, base_z_offset = self.pick_offsets[cmd]

        if cmd in (1, 4) and raw_tx < self.near_bearing_wheel_x_threshold_m:
            if cmd == 1:
                x_offset = self.wheel_near_pick_offset_x
            else:
                x_offset = self.bearing_near_pick_offset_x
        elif cmd == 2 and raw_tx <= self.near_x_threshold_m:
            x_offset = self.boltnut_near_low_x_pick_offset_x
        elif cmd == 3 and raw_tx <= self.near_gear_x_threshold_m and raw_ty > self.gear_near_high_y_threshold_m:
            x_offset = self.gear_near_high_y_pick_offset_x
        elif cmd == 3 and raw_tx <= self.near_gear_x_threshold_m:
            x_offset = self.gear_near_pick_offset_x
        else:
            x_offset = base_x_offset
        tx = raw_tx + x_offset

        if raw_ty > self.high_y_threshold_m:
            y_offset = self.high_y_pick_offset_y[cmd]
        elif raw_ty < self.low_y_threshold_m:
            y_offset = self.low_y_pick_offset_y[cmd]
        else:
            y_offset = base_y_offset
        ty = raw_ty + y_offset

        raw_tz = float(target['z'])
        if cmd == 1 and raw_tx < self.near_bearing_wheel_low_x_threshold_m:
            z_offset = self.wheel_near_low_x_pick_offset_z
        elif cmd == 4 and raw_tx < self.near_bearing_wheel_low_x_threshold_m:
            z_offset = self.bearing_near_low_x_pick_offset_z
        elif cmd == 2 and tx <= self.near_x_threshold_m:
            z_offset = self.boltnut_near_pick_offset_z
        elif cmd == 2 or tx <= self.near_x_threshold_h:
            z_offset = self.near_pick_offset_z[cmd]
        else:
            z_offset = base_z_offset
        tz = raw_tz + z_offset
        tz = max(self.pick_z_min, min(tz, self.pick_z_max))

        self.get_logger().info(
            f'depth pick target: raw=({raw_tx:.3f}, {raw_ty:.3f}, {raw_tz:.3f}), '
            f'pick=({tx:.3f}, {ty:.3f}, {tz:.3f}), '
            f'x_offset={x_offset:.3f}, y_offset={y_offset:.3f}, z_offset={z_offset:.3f}'
        )

        self.send_gripper_blocking(0.019)

        if not self.send_precise_goal_blocking(tx, ty, tz):
            return False

        self.send_gripper_blocking(-0.01)
        self.get_logger().info('그리퍼 클로즈 후 1.0초 대기')
        self.sleep_with_pause(0.3)

        if raw_tx <= self.near_x_threshold_m:
            lift_z = min(tz + self.near_after_pick_lift_height, self.pick_z_max)
            self.get_logger().info(f'x 가까운 물체 pick 후 z 리프트: {tz:.3f} -> {lift_z:.3f}')
            if not self.send_precise_goal_blocking(tx, ty, lift_z):
                self.get_logger().warn(f'x 가까운 물체 z 리프트 실패: target=({tx:.3f}, {ty:.3f}, {lift_z:.3f})')
                return False
        else:
            self.send_arm_joint_topic_blocking(
                self.after_pick_waypoint_joints,
                'pick 후 경유지 이동',
                self.place_transfer_motion_sec,
            )

        self.send_arm_joint_topic_blocking(
            self.place_hover_joints[cmd],
            f'{cmd}({self.command_name[cmd]}) 분류 상자 이동 성공 joints={self.place_hover_joints[cmd]}',
            self.place_transfer_motion_sec,
        )

        self.send_gripper_blocking(0.019)
        self.get_logger().info('상자 위치 도착 후 그리퍼 오픈, 0.5초 대기')
        self.sleep_with_pause(0.5)
        return True

    def send_arm_joint_topic(self, joint_degrees: List[float], duration_sec: float = 2.0):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in joint_degrees]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_arm_joint_topic_blocking(
        self,
        joint_degrees: List[float],
        label: str,
        duration_sec: float = 2.0,
    ) -> bool:
        while rclpy.ok():
            self.send_arm_joint_topic(joint_degrees, duration_sec)
            paused = self.wait_joint_motion(label, duration_sec)
            if not paused:
                return True
            self.get_logger().info(f'{label}: okay 확인 후 같은 joint 목표 재전송')
        return False

    def stop_arm_motion(self):
        with self.current_joint_lock:
            current_positions = list(self.current_joint_positions) if self.current_joint_positions is not None else None

        if current_positions is None:
            self.get_logger().warn('현재 joint_states가 없어 즉시 정지 trajectory를 보낼 수 없습니다.')
            return

        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = current_positions
        point.velocities = [0.0] * len(current_positions)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100_000_000

        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_precise_goal_blocking(self, x: float, y: float, z: float) -> bool:
        while rclpy.ok():
            result = self.send_precise_goal_once(x, y, z)
            if result is None:
                self.get_logger().info(
                    f'okay 확인 후 MoveIt 목표 재시도: target=({x:.3f}, {y:.3f}, {z:.3f})'
                )
                continue
            return result
        return False

    def send_precise_goal_once(self, x: float, y: float, z: float) -> Optional[bool]:
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 3.0

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.y = 0.707
        target_pose.orientation.w = 0.707

        constraints = Constraints()

        p_con = PositionConstraint()
        p_con.header.frame_id = self.goal_frame
        p_con.link_name = self.ee_link_name
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        p_con.constraint_region.primitives.append(box)
        p_con.constraint_region.primitive_poses.append(target_pose)
        constraints.position_constraints.append(p_con)

        if x < 0.216:
            tolerance = 50.0
        else:
            tolerance = 35.0

        o_con = OrientationConstraint()
        o_con.header.frame_id = self.goal_frame
        o_con.link_name = self.ee_link_name
        o_con.orientation = target_pose.orientation
        o_con.absolute_x_axis_tolerance = math.radians(tolerance)
        o_con.absolute_y_axis_tolerance = math.radians(tolerance)
        o_con.absolute_z_axis_tolerance = math.radians(180.0)
        o_con.weight = 1.0
        constraints.orientation_constraints.append(o_con)

        goal_msg.request.goal_constraints.append(constraints)

        future = self.arm_client.send_goal_async(goal_msg)
        while rclpy.ok() and not future.done():
            if not self.hand_pause_event.is_set():
                self.wait_if_hand_paused()
                return None
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if not self.hand_pause_event.is_set():
                cancel_future = goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    time.sleep(0.02)
                self.wait_if_hand_paused()
                return None
            time.sleep(0.05)

        result = result_future.result()
        error_val = getattr(getattr(result, 'result', None), 'error_code', None)
        error_val = getattr(error_val, 'val', 1)
        if error_val != 1:
            self.get_logger().error(f'MoveIt result error_code={error_val}')
            return False

        return True

    def send_gripper_blocking(self, position: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        future = self.gripper_client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            self.wait_if_hand_paused()
            time.sleep(0.05)
        self.sleep_with_pause(0.6)
        return True

    def wait_recognition(self):
        self.get_logger().info(f'인식안정화를 위해 {self.recognition_wait_sec:.1f}s 대기')
        self.sleep_with_pause(self.recognition_wait_sec)

    def wait_joint_motion(self, _label: str, wait_sec: Optional[float] = None):
        return self.sleep_with_pause(self.joint_motion_wait_sec if wait_sec is None else wait_sec)

    def sleep_with_pause(self, duration_sec: float):
        remaining = duration_sec
        paused = False
        while rclpy.ok() and remaining > 0.0:
            if not self.hand_pause_event.is_set():
                paused = True
                self.wait_if_hand_paused()
            sleep_time = min(0.05, remaining)
            time.sleep(sleep_time)
            remaining -= sleep_time
        return paused


def main():
    rclpy.init()
    node = TestNode6()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
