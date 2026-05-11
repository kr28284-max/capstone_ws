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
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestNode5(Node):
    def __init__(self):
        super().__init__('test_node5')

        self.callback_group = ReentrantCallbackGroup()
        self.ready = False
        self.busy = True

        self.goal_frame = 'base_link'
        self.ee_link_name = 'end_effector_link'

        self.recognition_wait_sec = 5.0
        self.detect_timeout_sec = 2.0
        self.joint_motion_wait_sec = 2.5

        self.approach_height = 0.12
        self.pick_z_min = 0.03
        self.pick_z_max = 0.12
        self.far_x_threshold_m = 0.230
        self.far_pick_z_raise_m = 0.015
        self.place_hover_joints = {
            1: [90.0, -50.0, 58.0, 59.0],
            2: [-135.0, -50.0, 58.0, 59.0],
            3: [-90.0, -50.0, 58.0, 59.0],
            4: [135.0, -50.0, 58.0, 59.0],
        }

        self.vision_joint_deg = [0.0, -6.0, -24.0, 108.0]
        self.home_joint_deg = [0.0, -6.0, -24.0, 108.0]
        self.hand_recognition_joint_deg = [0.0, -60.0, 20.0, 10.0]

        self.pick_offset_x = 0.055
        self.pick_offset_y = 0.0
        self.pick_offset_z = 0.005

        self.command_name = {
            1: 'bearing',
            2: 'boltnut',
            3: 'gear',
            4: 'wheel',
        }
        self.command_class_id = {
            1: 0,
            2: 1,
            3: 2,
            4: 3,
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
        self.hand_sub = self.create_subscription(
            Int32,
            '/vision/hand_finger_count',
            self.hand_finger_callback,
            10,
            callback_group=self.callback_group,
        )

        self.latest_detections: List[Dict] = []
        self.latest_detections_stamp = 0.0
        self.det_lock = threading.Lock()
        self.last_finger = -1
        self.same_finger_count = 0
        self.required_stable_count = 12
        self.last_hand_trigger_time = 0.0
        self.hand_trigger_cooldown_sec = 2.0

        self.get_logger().info('START test_node5: test_node3 + test_sequence 통합')
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

    def hand_finger_callback(self, msg: Int32):
        finger = int(msg.data)

        # 손이 인식 안되거나 유효 범위 밖이면 안정화 카운터 리셋
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

        # 동일 손가락 유지 상태에서 중복 발사를 막기 위해 콜백 카운터 리셋
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
        threading.Thread(target=self.run_command_sequence, args=(cmd,), daemon=True).start()

    def run_command_sequence(self, cmd: int):
        name = self.command_name[cmd]
        count = 0
        self.get_logger().info(f'명령 {cmd}({name}) 처리 시작')
        try:
            while rclpy.ok():
                self.send_arm_joint_topic(self.vision_joint_deg)
                self.wait_recognition()

                target = self.find_leftmost_target(cmd, self.detect_timeout_sec)
                if target is None:
                    self.get_logger().info(f'{name} 추가 물체 없음. 총 {count}개 처리 후 종료')
                    break

                ok = self.execute_enhanced_sequence(cmd, target)
                if not ok:
                    self.get_logger().warn('시퀀스 실패로 중단')
                    break
                count += 1
        finally:
            self.send_arm_joint_topic(self.hand_recognition_joint_deg)
            self.wait_joint_motion('손가락 인식자세 복귀')
            self.busy = False
            self.get_logger().info('FINISH 다음 명령 대기')

    def find_leftmost_target(self, cmd: int, timeout_sec: float) -> Optional[Dict]:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
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
        tx = float(target['x']) + self.pick_offset_x
        ty = float(target['y']) + self.pick_offset_y
        tz = float(target['z']) + self.pick_offset_z
        tz = max(self.pick_z_min, min(tz, self.pick_z_max))
        if tx >= self.far_x_threshold_m:
            tz = min(tz + self.far_pick_z_raise_m, self.pick_z_max)

        approach_z = min(tz + self.approach_height, 0.22)

        self.send_gripper_blocking(0.019)

        if not self.send_precise_goal_blocking(tx, ty, approach_z):
            return False
        if not self.send_precise_goal_blocking(tx, ty, tz):
            return False

        self.send_gripper_blocking(-0.01)

        if not self.send_precise_goal_blocking(tx, ty, approach_z):
            return False

        self.send_arm_joint_topic(self.place_hover_joints[cmd])
        self.wait_joint_motion('분류 상자 상공 이동')

        self.send_gripper_blocking(0.019)
        return True

    def send_arm_joint_topic(self, joint_degrees: List[float]):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in joint_degrees]
        point.time_from_start.sec = 2

        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_precise_goal_blocking(self, x: float, y: float, z: float) -> bool:
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

        o_con = OrientationConstraint()
        o_con.header.frame_id = self.goal_frame
        o_con.link_name = self.ee_link_name
        o_con.orientation = target_pose.orientation
        o_con.absolute_x_axis_tolerance = math.radians(35.0)
        o_con.absolute_y_axis_tolerance = math.radians(35.0)
        o_con.absolute_z_axis_tolerance = math.radians(180.0)
        o_con.weight = 1.0
        constraints.orientation_constraints.append(o_con)

        goal_msg.request.goal_constraints.append(constraints)

        future = self.arm_client.send_goal_async(goal_msg)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
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
            time.sleep(0.05)
        time.sleep(0.6)
        return True

    def wait_recognition(self):
        self.get_logger().info(f'인식 안정화를 위해 {self.recognition_wait_sec:.1f}s 대기')
        time.sleep(self.recognition_wait_sec)

    def wait_joint_motion(self, _label: str):
        time.sleep(self.joint_motion_wait_sec)


def main():
    rclpy.init()
    node = TestNode5()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
