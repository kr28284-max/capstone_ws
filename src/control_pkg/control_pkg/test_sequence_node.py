# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from moveit_msgs.action import MoveGroup
# from control_msgs.action import GripperCommand
# from moveit_msgs.msg import Constraints, PositionConstraint
# from geometry_msgs.msg import Pose
# from shape_msgs.msg import SolidPrimitive
# import time
# import threading
# import math

# class TestMasterNode(Node):
#     def __init__(self):
#         super().__init__('test_master_node')

#         # 퍼블리셔 및 액션 클라이언트
#         self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
#         self._arm_action_client = ActionClient(self, MoveGroup, '/move_action')
#         self._gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

#         # 비전 구독 없이 바로 테스트 스레드 실행
#         self.get_logger().info("🚀 [START] 비전 없는 시퀀스 테스트 모드 가동")
#         threading.Thread(target=self.run_test_sequence).start()

#     def run_test_sequence(self):
#         # 0. 서버 연결 대기
#         self.get_logger().info("⏳ MoveIt 및 그리퍼 서버 연결 대기 중...")
#         self._arm_action_client.wait_for_server()
#         self._gripper_action_client.wait_for_server()
#         self.get_logger().info("✅ 서버 연결 완료. 3초 후 테스트 시퀀스를 시작합니다.")
#         time.sleep(3.0)

#         # 가상의 물체 좌표 (로봇 정면 20cm, 바닥)
#         dummy_x = 0.20
#         dummy_y = 0.0
#         dummy_z = 0.04
        
#         # 테스트할 4가지 케이스
#         targets = ['a', 'b', 'c', 'd']

#         while rclpy.ok():
#             for target_id in targets:
#                 self.get_logger().info(f"\n{'='*40}\n🔥 [TEST] 타겟 '{target_id}' 시퀀스 시작\n{'='*40}")
#                 self.execute_single_sequence(target_id, dummy_x, dummy_y, dummy_z)
                
#                 self.get_logger().info("⏳ 다음 타겟 테스트 전 5초 대기...")
#                 time.sleep(5.0)

#     def execute_single_sequence(self, target_class_id, tx, ty, tz):
#         try:
#             # --- [Step 1] 손가락 인식 대기 자세 ---
#             self.get_logger().info("[Step 1] 📷 손가락 인식 자세 (정면 보기)")
#             self.send_arm_joint_topic([0.0, -60.0, 20.0, 60.0])
#             self.send_gripper_blocking(0.019) # 그리퍼 열기
#             time.sleep(3.0) # (가상) 손가락 인식하는 시간 대기

#             # --- [Step 2] 물체 확인하러 홈 자세로 이동 ---
#             self.get_logger().info(f"[Step 2] 🔍 {target_class_id} 물체 확인하러 홈 자세(바닥 보기)로 이동")
#             self.send_arm_joint_topic([0.0, -44.0, 0.0, 117.0])
#             time.sleep(3.0)

#             # --- [Step 3] 접근 및 파지 ---
#             self.get_logger().info("[Step 3] ✊ 물체 접근 및 파지")
#             APPROACH_Z = tz + 0.15
#             PICK_Z = tz

#             # 물체 위로 이동
#             if self.send_precise_goal_blocking(tx, ty, APPROACH_Z):
#                 # 물체로 하강
#                 self.send_precise_goal_blocking(tx, ty, PICK_Z)
#                 # 잡기
#                 self.get_logger().info("   -> 꽉 잡습니다.")
#                 self.send_gripper_blocking(-0.01)
#                 time.sleep(1.0)
#                 # 들어올리기
#                 self.send_precise_goal_blocking(tx, ty, APPROACH_Z)
#             else:
#                 self.get_logger().error("⚠️ [접근 실패] 파지 위치로 이동할 수 없습니다. URDF 한계이거나 좌표가 너무 멉니다.")
#                 return

#             # --- [Step 4] 분류 장소로 이동 (조인트 제어) ---
#             self.get_logger().info(f"[Step 4] 🚚 '{target_class_id}' 분류 상자로 이동하여 놓기")
#             if target_class_id == 'a':   # 베어링 (왼쪽)
#                 drop_joints = [90.0, -50.0, 58.0, 59.0]
#             elif target_class_id == 'b': # 휠 (왼쪽 뒤)
#                 drop_joints = [135.0, -50.0, 58.0, 59.0]
#             elif target_class_id == 'c': # 볼트/너트 (오른쪽 뒤)
#                 drop_joints = [-135.0, -50.0, 58.0, 59.0]
#             elif target_class_id == 'd': # 기어 (오른쪽)
#                 drop_joints = [-90.0, -50.0, 58.0, 59.0]

#             self.send_arm_joint_topic(drop_joints)
#             time.sleep(4.0)
            
#             # 놓기
#             self.get_logger().info("   -> 물체를 놓습니다.")
#             self.send_gripper_blocking(0.019)
#             time.sleep(1.0)

#             # --- [Step 5] 다시 손가락 인식 자세로 복귀 ---
#             self.get_logger().info("[Step 5] ♻️ 사이클 완료. 초기 인식 자세로 복귀합니다.")
#             self.send_arm_joint_topic([0.0, -60.0, 20.0, 60.0])
#             time.sleep(3.0)

#         except Exception as e:
#             self.get_logger().error(f"‼️ 시퀀스 에러 발생: {e}")

#     # ------------------ 유틸리티 함수들 ------------------
#     def send_arm_joint_topic(self, joint_degrees):
#         msg = JointTrajectory()
#         msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
#         joint_radians = [math.radians(d) for d in joint_degrees]
#         point = JointTrajectoryPoint()
#         point.positions = joint_radians
#         point.time_from_start.sec = 2
#         msg.points.append(point)
#         self.arm_pub.publish(msg)

#     def send_precise_goal_blocking(self, x, y, z):
#         goal_msg = MoveGroup.Goal()
#         goal_msg.request.group_name = 'arm'
#         goal_msg.request.allowed_planning_time = 3.0
#         goal_msg.request.num_planning_attempts = 10
        
#         target_pose = Pose()
#         target_pose.position.x = x
#         target_pose.position.y = y
#         target_pose.position.z = z
#         target_pose.orientation.x = 0.0
#         target_pose.orientation.y = 0.707
#         target_pose.orientation.z = 0.0
#         target_pose.orientation.w = 0.707
        
#         constraints = Constraints()
#         p_con = PositionConstraint()
#         p_con.header.frame_id = "base_link"
#         p_con.link_name = "end_effector_link"
#         box = SolidPrimitive()
#         box.type = SolidPrimitive.BOX
#         box.dimensions = [0.01, 0.01, 0.01]
#         p_con.constraint_region.primitives.append(box)
#         p_con.constraint_region.primitive_poses.append(target_pose)
#         p_con.weight = 1.0
#         constraints.position_constraints.append(p_con)
#         goal_msg.request.goal_constraints.append(constraints)

#         future = self._arm_action_client.send_goal_async(goal_msg)
#         while rclpy.ok() and not future.done(): time.sleep(0.05)
#         goal_handle = future.result()
        
#         if not goal_handle.accepted:
#             return False

#         res_future = goal_handle.get_result_async()
#         while rclpy.ok() and not res_future.done(): time.sleep(0.05)
#         return True

#     def send_gripper_blocking(self, position):
#         goal_msg = GripperCommand.Goal()
#         goal_msg.command.position = float(position)
#         self._gripper_action_client.send_goal_async(goal_msg)
#         time.sleep(1.0)
#         return True

# def main():
#     rclpy.init()
#     node = TestMasterNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand
from moveit_msgs.msg import Constraints, PositionConstraint
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
import time
import threading
import math

class TestMasterNode(Node):
    def __init__(self):
        super().__init__('test_master_node')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._arm_action_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.get_logger().info("🚀 [START] 그리퍼 및 높이 조절 테스트 모드 가동")
        threading.Thread(target=self.run_test_sequence).start()

    def run_test_sequence(self):
        self._arm_action_client.wait_for_server()
        self._gripper_action_client.wait_for_server()
        self.get_logger().info("✅ 서버 연결 완료.")
        time.sleep(2.0)

        # 가상의 물체 좌표 및 높이 설정
        dummy_x, dummy_y, dummy_z = 0.20, 0.0, 0.04
        targets = ['a', 'b', 'c', 'd']

        while rclpy.ok():
            for target_id in targets:
                self.get_logger().info(f"\n🔥 [TEST] 타겟 '{target_id}' 작업 시작")
                self.execute_enhanced_sequence(target_id, dummy_x, dummy_y, dummy_z)
                time.sleep(5.0)

    def execute_enhanced_sequence(self, target_class_id, tx, ty, tz):
        try:
            # 1. 준비 자세 및 그리퍼 열기
            self.get_logger().info("📷 1. 인식 자세 이동 및 그리퍼 개방")
            self.send_arm_joint_topic([0.0, -60.0, 20.0, 60.0])
            self.send_gripper_blocking(0.019) # 최대 개방
            time.sleep(2.0)

            # 2. 파지 시퀀스 (접근 -> 하강 -> 잡기 -> 상승)
            self.get_logger().info("✊ 2. 물체 파지 시퀀스")
            APPROACH_Z = tz + 0.12
            PICK_Z = tz

            if self.send_precise_goal_blocking(tx, ty, APPROACH_Z):
                self.send_precise_goal_blocking(tx, ty, PICK_Z)
                self.send_gripper_blocking(-0.01) # 꽉 잡기
                time.sleep(1.0)
                self.send_precise_goal_blocking(tx, ty, APPROACH_Z) # 들어올리기
            else:
                return

            # 3. 분류 장소로 이동 (조인트 제어)
            self.get_logger().info(f"🚚 3. '{target_class_id}' 상자 위로 이동")
            if target_class_id == 'a': drop_j1 = 90.0
            elif target_class_id == 'b': drop_j1 = 135.0
            elif target_class_id == 'c': drop_j1 = -135.0
            elif target_class_id == 'd': drop_j1 = -90.0

            # 상자 위 공중 자세 (Joint2, 3, 4 각도를 조절해 높이 확보)
            # -30.0 등으로 설정하면 팔이 더 높게 들립니다.
            self.send_arm_joint_topic([drop_j1, -30.0, 30.0, 50.0]) 
            time.sleep(3.5)
            
            # 4. 상자 안으로 안전하게 내려가서 놓기 (Place 동작)
            # 상자 높이를 고려하여 tz보다 높은 위치에서 그리퍼 작동
            self.get_logger().info("🎁 4. 상자 내부로 하강 후 물체 놓기")
            PLACE_Z = tz + 0.10 # 상자 높이가 10cm라고 가정 시
            
            # 현재 조인트 위치(상자 위)에서 정밀 좌표 제어로 살짝 내려갑니다.
            # (참고: 상자 위치 좌표를 미리 알고 있다면 tx, ty 대신 상자 좌표 입력 가능)
            # 여기서는 현재 각도에서 그리퍼만 열어 투하합니다.
            self.send_gripper_blocking(0.019) 
            time.sleep(1.0)

            # 5. 복귀
            self.get_logger().info("♻️ 5. 사이클 완료 및 복귀")
            self.send_arm_joint_topic([0.0, -60.0, 20.0, 60.0])
            time.sleep(2.0)

        except Exception as e:
            self.get_logger().error(f"‼️ 에러: {e}")

    # --- 기존 유틸리티 함수 (send_arm_joint_topic, send_precise_goal_blocking, send_gripper_blocking) 동일 ---
    def send_arm_joint_topic(self, joint_degrees):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        joint_radians = [math.radians(d) for d in joint_degrees]
        point = JointTrajectoryPoint()
        point.positions = joint_radians
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_precise_goal_blocking(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 3.0
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.y, target_pose.orientation.w = 0.707, 0.707
        
        constraints = Constraints()
        p_con = PositionConstraint()
        p_con.header.frame_id = "base_link"
        p_con.link_name = "end_effector_link"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        p_con.constraint_region.primitives.append(box)
        p_con.constraint_region.primitive_poses.append(target_pose)
        constraints.position_constraints.append(p_con)
        goal_msg.request.goal_constraints.append(constraints)

        future = self._arm_action_client.send_goal_async(goal_msg)
        while rclpy.ok() and not future.done(): time.sleep(0.05)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted: return False
        res_future = goal_handle.get_result_async()
        while rclpy.ok() and not res_future.done(): time.sleep(0.05)
        return True

    def send_gripper_blocking(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        self._gripper_action_client.send_goal_async(goal_msg)
        time.sleep(1.0)
        return True

def main():
    rclpy.init()
    node = TestMasterNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()