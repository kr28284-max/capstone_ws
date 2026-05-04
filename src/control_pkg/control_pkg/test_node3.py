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


class TestNode3(Node):
    def __init__(self):
        super().__init__("test_node3")

        self.callback_group = ReentrantCallbackGroup()
        self.busy = True
        self.ready = False

        # Timing
        # 요청사항: 5초 대기는 "인식 단계"에서만 사용
        self.recognition_wait_sec = 5.0
        self.joint_motion_wait_sec = 2.2
        self.detect_timeout_sec = 2.0

        # Frames and motion
        self.goal_frame = "base_link"
        self.ee_link_name = "end_effector_link"
        self.approach_height = 0.10
        self.pick_z_min = 0.03
        self.pick_z_max = 0.12
        # Far target compensation: if x >= 230mm, keep pick a bit higher.
        self.far_x_threshold_m = 0.230
        self.far_pick_z_raise_m = 0.015
        # True: 인식 자세에서 그리퍼만 open/close (무브잇 이동 없음)
        self.gripper_only_mode = False
        # 픽 자세: 수직 기준으로 기울기 허용 (±35도)
        self.use_orientation_constraint = True
        self.vertical_tilt_tolerance_rad = math.radians(35.0)

        # Pick compensation (새 인식 자세 [0, -6, -24, 108] 기준 초기값)
        self.pick_offset_x = 0.045
        self.pick_offset_y = 0.0
        self.pick_offset_z = 0.005

        # Poses
        # self.vision_joint_deg = [0.0, -21.0, -8.0, 112.0]
        # self.home_joint_deg = [0.0, -21.0, -8.0, 112.0]
        self.vision_joint_deg = [0.0, -6.0, -24.0, 108.0]
        self.home_joint_deg = [0.0, -6.0, -24.0, 108.0]
        self.drop_joint_by_cmd = {
            1: [90.0, -50.0, 58.0, 59.0],    # bearing
            2: [-135.0, -50.0, 58.0, 59.0],  # bolt
            3: [-90.0, -50.0, 58.0, 59.0],   # gear
            4: [135.0, -50.0, 58.0, 59.0],   # wheel
        }

        # Command map (best.pt actual order):
        # 0:bearing, 1:boltnut, 2:gear, 3:wheel
        self.command_name = {
            1: "bearing",
            2: "boltnut",
            3: "gear",
            4: "wheel",
        }
        # 사용자 명령(1~4) -> YOLO class_id(0-based)
        self.command_class_id = {
            1: 0,
            2: 1,
            3: 2,
            4: 3,
        }

        # ROS interfaces
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.arm_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.callback_group)
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
            callback_group=self.callback_group,
        )

        # Vision detections topic (JSON string)
        self.detection_sub = self.create_subscription(
            String,
            "/vision/detections",
            self.detection_callback,
            10,
            callback_group=self.callback_group,
        )
        # vision_master_node5 환경 호환 토픽(백업)
        self.detection_sub_alt = self.create_subscription(
            String,
            "/vision/detections5",
            self.detection_callback,
            10,
            callback_group=self.callback_group,
        )
        # Pick command topic: 1=bearing, 2=boltnut, 3=gear, 4=wheel
        self.command_sub = self.create_subscription(
            Int32,
            "/pick_command",
            self.command_callback,
            10,
            callback_group=self.callback_group,
        )

        self.latest_detections: List[Dict] = []
        self.latest_detections_stamp = 0.0
        self.last_detection_warn_time = 0.0
        self.det_lock = threading.Lock()

        self.get_logger().info("🚀 [START] test_node3 시작: /pick_command 대기")
        threading.Thread(target=self.init_robot_sequence, daemon=True).start()
        threading.Thread(target=self.keyboard_input_loop, daemon=True).start()

    def init_robot_sequence(self):
        self.get_logger().info("⏳ MoveIt/Gripper 서버 연결 대기")
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ /move_action 서버 연결 실패")
            return
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ /gripper_controller/gripper_cmd 서버 연결 실패")
            return

        self.get_logger().info("✅ 서버 연결 완료. 초기 자세 이동")
        self.send_arm_joint_topic(self.home_joint_deg)
        self.wait_joint_motion("초기 자세 이동")
        self.send_gripper_blocking(-0.01)

        self.ready = True
        self.busy = False
        self.get_logger().info(
            "🎯 [READY] 명령 대기: 키보드(1~4 + Enter) 또는 "
            "ros2 topic pub /pick_command std_msgs/msg/Int32 '{data: 1}'"
        )
        if self.gripper_only_mode:
            self.get_logger().info("🧪 현재 모드: 인식자세 그리퍼 테스트(열기/닫기만 수행)")
        else:
            self.get_logger().info("🤖 현재 모드: 일반 Pick&Place")

    def detection_callback(self, msg: String):
        try:
            parsed = json.loads(msg.data)
            dets = parsed.get("detections", [])
            if not isinstance(dets, list):
                return

            normalized = []
            for d in dets:
                if not isinstance(d, dict):
                    continue
                normalized.append(
                    {
                        "class_id": int(d.get("class_id", -1)),
                        "class_name": str(d.get("class_name", "")).lower(),
                        "u": float(d.get("u", 1e9)),
                        "x": float(d.get("x", 0.0)),
                        "y": float(d.get("y", 0.0)),
                        "z": float(d.get("z", 0.0)),
                    }
                )

            with self.det_lock:
                self.latest_detections = normalized
                self.latest_detections_stamp = time.time()
            if normalized:
                self.get_logger().debug(
                    f"👁️ detections 수신: {len(normalized)}개, 첫 물체="
                    f"{normalized[0]['class_name']} (id={normalized[0]['class_id']})"
                )
        except Exception:
            return

    def command_callback(self, msg: Int32):
        cmd = int(msg.data)
        self.request_command(cmd, source="topic")

    def request_command(self, cmd: int, source: str):
        if cmd not in self.command_name:
            self.get_logger().warn(f"⚠️ [{source}] 지원하지 않는 명령: {cmd} (허용: 1~4)")
            return
        if not self.ready:
            self.get_logger().warn(f"⚠️ [{source}] 아직 초기화 중입니다.")
            return
        if self.busy:
            self.get_logger().warn(f"⚠️ [{source}] 현재 작업 중이라 새 명령을 무시합니다.")
            return

        self.busy = True
        threading.Thread(target=self.run_command_sequence, args=(cmd,), daemon=True).start()

    def keyboard_input_loop(self):
        print("\n[Keyboard] 1=bearing, 2=boltnut, 3=gear, 4=wheel, q=quit")
        while rclpy.ok():
            try:
                raw = input("명령 입력 (1~4, q): ").strip().lower()
            except EOFError:
                return
            except Exception:
                continue

            if raw == "":
                continue
            if raw in ("q", "quit", "exit"):
                self.get_logger().info("🛑 키보드 종료 명령 수신. 노드를 종료합니다.")
                rclpy.shutdown()
                return
            if raw in ("1", "2", "3", "4"):
                self.request_command(int(raw), source="keyboard")
                continue

            self.get_logger().warn("⚠️ 키보드 입력은 1,2,3,4 또는 q만 지원합니다.")

    def run_command_sequence(self, cmd: int):
        name = self.command_name[cmd]
        self.get_logger().info(f"🧭 명령 수신: {cmd}번({name}) 전량 처리 시작")
        count = 0
        try:
            while rclpy.ok():
                self.get_logger().info(f"👀 [{cmd}-{count+1}] 인식 자세 이동")
                self.send_arm_joint_topic(self.vision_joint_deg)
                self.wait_recognition()

                target = self.find_leftmost_target(cmd, timeout_sec=self.detect_timeout_sec)
                if target is None:
                    self.get_logger().info(f"✅ {cmd}번({name}) 추가 물체 없음. 처리 종료 (총 {count}개).")
                    break

                self.get_logger().info(
                    f"🎯 타겟 선택(가장 왼쪽): class={target['class_name']} "
                    f"u={target['u']:.1f}, xyz=({target['x']:.3f}, {target['y']:.3f}, {target['z']:.3f})"
                )

                if not self.execute_pick_and_place(cmd, target):
                    self.get_logger().warn("⚠️ Pick&Place 실패로 명령을 중단합니다.")
                    break

                count += 1
                if self.gripper_only_mode:
                    self.get_logger().info("🧪 그리퍼 테스트 모드이므로 1회 수행 후 종료")
                    break
        except Exception as exc:
            self.get_logger().error(f"‼️ 시퀀스 예외: {exc}")
        finally:
            self.send_arm_joint_topic(self.home_joint_deg)
            self.wait_joint_motion("홈 복귀")
            self.busy = False
            self.get_logger().info("✨ [FINISH] 다음 /pick_command 대기")

    def find_leftmost_target(self, cmd: int, timeout_sec: float) -> Optional[Dict]:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            with self.det_lock:
                dets = list(self.latest_detections)
                stamp = self.latest_detections_stamp

            # 프레임 지연/큐 지연 대응: 0.8s -> 2.0s로 완화
            if dets and (time.time() - stamp) < 2.0:
                candidates = [d for d in dets if self.is_command_match(cmd, d)]
                if candidates:
                    candidates.sort(key=lambda d: d["u"])  # leftmost in image
                    return candidates[0]
            time.sleep(0.05)
        now = time.time()
        if now - self.last_detection_warn_time > 1.0:
            self.get_logger().warn(
                "⚠️ 감지 토픽 수신이 없거나 오래됨. "
                "vision_master_node5 실행/빌드/source 상태와 "
                "/vision/detections(또는 /vision/detections5) 발행을 확인하세요."
            )
            self.last_detection_warn_time = now
        return None

    def is_command_match(self, cmd: int, det: Dict) -> bool:
        class_name = det.get("class_name", "")
        expected = self.command_name[cmd]
        # 엄격 매칭:
        # 1) class_name 정확 일치 또는
        # 2) class_id 정확 일치
        if class_name == expected:
            return True
        class_id = int(det.get("class_id", -1))
        return class_id == self.command_class_id[cmd]

    def execute_pick_and_place(self, cmd: int, target: Dict) -> bool:
        tx = float(target["x"]) + self.pick_offset_x
        ty = float(target["y"]) + self.pick_offset_y
        tz_raw = float(target["z"]) + self.pick_offset_z
        tz = max(self.pick_z_min, min(tz_raw, self.pick_z_max))
        if tx >= self.far_x_threshold_m:
            tz = min(tz + self.far_pick_z_raise_m, self.pick_z_max)
        approach_z = min(tz + self.approach_height, 0.22)

        self.get_logger().info(
            f"📏 보정 좌표: raw=({target['x']:.3f}, {target['y']:.3f}, {target['z']:.3f}) -> "
            f"pick=({tx:.3f}, {ty:.3f}, {tz:.3f}), approach_z={approach_z:.3f}"
        )

        if self.gripper_only_mode:
            self.get_logger().info("🧪 인식자세 유지: 그리퍼 열기/닫기만 수행")
            self.send_gripper_blocking(0.019)
            self.send_gripper_blocking(-0.01)
            self.send_gripper_blocking(0.019)
            return True

        self.send_gripper_blocking(0.019)

        if not self.send_precise_goal_blocking(tx, ty, approach_z):
            return False

        if not self.send_precise_goal_blocking(tx, ty, tz):
            return False

        self.send_gripper_blocking(-0.01)

        if not self.send_precise_goal_blocking(tx, ty, approach_z):
            return False

        drop_joint = self.drop_joint_by_cmd[cmd]
        self.send_arm_joint_topic(drop_joint)
        self.wait_joint_motion("분류 위치 이동")

        self.send_gripper_blocking(0.019)
        return True

    def wait_recognition(self):
        self.get_logger().info(f"⏱️ 인식 안정화를 위해 {self.recognition_wait_sec:.1f}s 대기")
        time.sleep(self.recognition_wait_sec)

    def wait_joint_motion(self, label: str):
        self.get_logger().info(f"⏱️ {label} 완료 대기 {self.joint_motion_wait_sec:.1f}s")
        time.sleep(self.joint_motion_wait_sec)

    def send_arm_joint_topic(self, joint_degrees: List[float]):
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in joint_degrees]
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_precise_goal_blocking(self, x: float, y: float, z: float) -> bool:
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.allowed_planning_time = 3.0
        goal_msg.request.num_planning_attempts = 10

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.707
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.707

        constraints = Constraints()
        p_con = PositionConstraint()
        p_con.header.frame_id = self.goal_frame
        p_con.link_name = self.ee_link_name
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.02, 0.02, 0.02]
        p_con.constraint_region.primitives.append(box)
        p_con.constraint_region.primitive_poses.append(target_pose)
        p_con.weight = 1.0
        constraints.position_constraints.append(p_con)

        if self.use_orientation_constraint:
            # Keep gripper vertical (tool pointing downward) during pick motion.
            o_con = OrientationConstraint()
            o_con.header.frame_id = self.goal_frame
            o_con.link_name = self.ee_link_name
            o_con.orientation = target_pose.orientation
            # Vertical +/- 25deg allowance.
            o_con.absolute_x_axis_tolerance = self.vertical_tilt_tolerance_rad
            o_con.absolute_y_axis_tolerance = self.vertical_tilt_tolerance_rad
            # Yaw는 과도 제한 시 플래닝 실패가 늘어 조금 더 여유를 둡니다.
            o_con.absolute_z_axis_tolerance = 1.0
            o_con.weight = 1.0
            constraints.orientation_constraints.append(o_con)

        goal_msg.request.goal_constraints.append(constraints)

        send_future = self.arm_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("🚫 MoveIt goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.05)
        return True

    def send_gripper_blocking(self, position: float) -> bool:
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        self.gripper_client.send_goal_async(goal_msg)
        time.sleep(1.0)
        return True


def main():
    rclpy.init()
    node = TestNode3()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
