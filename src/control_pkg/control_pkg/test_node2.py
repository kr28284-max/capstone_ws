# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.callback_groups import ReentrantCallbackGroup
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from moveit_msgs.action import MoveGroup
# from control_msgs.action import GripperCommand
# from moveit_msgs.msg import Constraints, PositionConstraint
# from geometry_msgs.msg import Pose, Point
# from shape_msgs.msg import SolidPrimitive
# import time
# import threading
# import math

# class MasterNode(Node):
#     def __init__(self):
#         super().__init__('master_node')
#         self.callback_group = ReentrantCallbackGroup()
#         self.is_busy = True # 초기화 완료 전 대기

#         # 퍼블리셔 및 액션 클라이언트
#         self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
#         self._arm_action_client = ActionClient(
#             self, 
#             MoveGroup, 
#             '/move_action', callback_group=self.callback_group)
#         self._gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd', callback_group=self.callback_group)

#         # 구독자 (비전 노드에서 좌표 수신)
#         self.target_sub = self.create_subscription(
#             Point, '/aruco_target_point', self.target_callback, 10, callback_group=self.callback_group)

#         self.get_logger().info("🚀 마스터 노드 시작: 로봇 초기화 중...")
#         threading.Thread(target=self.init_robot_sequence).start()

#     def init_robot_sequence(self):
#         # [수정] 어디서 대기 중인지 로그로 확인
#         self.get_logger().info("⏳ 팔(Arm) 액션 서버 대기 중... (/move_action)")
#         # 5초만 기다려보고 안 되면 에러 내기
#         if not self._arm_action_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("❌ 팔 액션 서버를 찾을 수 없습니다! 이름을 확인하세요.")
#             return

#         self.get_logger().info("⏳ 그리퍼(Gripper) 액션 서버 대기 중...")
#         if not self._gripper_action_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("❌ 그리퍼 액션 서버를 찾을 수 없습니다!")
#             return

#         self.get_logger().info("🚀 모든 서버 연결 완료! 초기 포즈로 이동합니다.")
#         time.sleep(1)
#         self.send_arm_joint_topic([0.0, -44.0, 0.0, 115.0])
#         self.send_gripper_blocking(-0.01)
#         time.sleep(3.5)
#         self.is_busy = False
#         self.get_logger().info("✅ 준비 완료. 목표 좌표 수신 대기 중.")



#     def target_callback(self, msg):
#         # 로봇이 작업 중이 아닐 때만 시퀀스 실행
#         if not self.is_busy:
#             self.is_busy = True
#             threading.Thread(target=self.execute_sequence_thread, args=(msg.x, msg.y, msg.z)).start()

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
#         self.get_logger().info(f"--- 이동 시도 --- X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}")

#         goal_msg = MoveGroup.Goal()
#         goal_msg.request.group_name = 'arm'
#         goal_msg.request.allowed_planning_time = 5.0
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
#         box.dimensions = [0.02, 0.02, 0.02]
#         p_con.constraint_region.primitives.append(box)
#         p_con.constraint_region.primitive_poses.append(target_pose)
#         p_con.weight = 1.0
#         constraints.position_constraints.append(p_con)
#         goal_msg.request.goal_constraints.append(constraints)

#         future = self._arm_action_client.send_goal_async(goal_msg)
#         while rclpy.ok() and not future.done(): time.sleep(0.1)
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("계획 실패: 목표 지점에 도달 불가능")
#             return False

#         res_future = goal_handle.get_result_async()
#         while rclpy.ok() and not res_future.done(): time.sleep(0.1)
#         return True

#     def send_gripper_blocking(self, position):
#         goal_msg = GripperCommand.Goal()
#         goal_msg.command.position = float(position)
#         self._gripper_action_client.send_goal_async(goal_msg)
#         time.sleep(1.5)
#         return True

#     def execute_sequence_thread(self, tx, ty, tz):
#         try:
            
#             PICK_Z = 0.04
#             APPROACH_Z = PICK_Z + 0.10
#             self.send_gripper_blocking(0.019) 
#             if self.send_precise_goal_blocking(tx, ty, APPROACH_Z):
#                 if self.send_precise_goal_blocking(tx, ty, PICK_Z):
#                     self.send_gripper_blocking(-0.01) 
#                     self.send_precise_goal_blocking(tx, ty, APPROACH_Z) 
            
#             self.send_arm_joint_topic([-117.0, -50.0, 58.0, 59.0])
#             time.sleep(4.0)
            
#             # self.send_gripper_blocking(0.019)
#             self.send_arm_joint_topic([0.0, -44.0, 0.0, 115.0])
#             time.sleep(3.0)
#         finally:
#             self.is_busy = False

# def main():
#     rclpy.init()
#     node = MasterNode()
#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node)
#     try:
#         executor.spin()
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from geometry_msgs.msg import Pose, Point, PointStamped
from shape_msgs.msg import SolidPrimitive
import time
import threading
import math
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.callback_group = ReentrantCallbackGroup()
        self.is_busy = True # 초기화 완료 전까지 target 무시
        self.source_frame = 'camera_rgb_optical_frame'
        self.target_frame = 'base_link'
        # Empirical base-frame pick compensation (meters).
        # Apply after camera->base transform for intuitive axis tuning.
        self.base_pick_offset_x = -0.15 # 기존 -0.05 > -0.17 즉, 로봇이 12cm 더 앞으로 간 것을 보정
        self.base_pick_offset_y = 0.05 # 기존 0.12 > 0.06 즉, 로봇이 6cm 더 왼쪽으로 간 것을 보정
        self.base_pick_offset_z = -0.03 # 기존 -0.02 > 0.02 즉, 로봇이 2cm 더 낮게 내려간 것을 보정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 1. 퍼블리셔 및 액션 클라이언트 설정
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # [수정] 액션 서버 이름을 절대 경로('/move_action')로 명확히 지정
        self._arm_action_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.callback_group)
        self._gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd', callback_group=self.callback_group)

        # 2. 구독자 (비전 노드에서 좌표 수신)
        self.target_sub = self.create_subscription(
            Point, '/aruco_target_point', self.target_callback, 10, callback_group=self.callback_group)

        self.get_logger().info("🚀 [START] 마스터 노드 가동 - 초기화 시퀀스 시작")
        threading.Thread(target=self.init_robot_sequence).start()

    def init_robot_sequence(self):
        self.get_logger().info("⏳ 서버 연결 대기 중...")
        
        if not self._arm_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ 실패: /move_action 서버를 찾을 수 없습니다. MoveIt이 켜져 있나요?")
            return
        
        if not self._gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ 실패: 그리퍼 서버를 찾을 수 없습니다. 하드웨어가 연결되었나요?")
            return

        self.get_logger().info("✅ 서버 연결 완료. 초기 포즈(Home) 이동 중...")
        # 초기 자세 (홈 자세)
        self.send_arm_joint_topic([0.0, -21.0, -8.0, 112.0])
        self.send_gripper_blocking(-0.01) # 그리퍼 살짝 닫기
        time.sleep(3.5)
        
        self.is_busy = False
        self.get_logger().info("🎯 [READY] 모든 준비 완료! 비전 좌표를 기다립니다.")

    def target_callback(self, msg):
        if self.is_busy:
            return # 현재 작업 중이면 새로운 좌표 무시

        cam_point = PointStamped()
        cam_point.header.stamp = self.get_clock().now().to_msg()
        cam_point.header.frame_id = self.source_frame
        cam_point.point.x = float(msg.x)
        cam_point.point.y = float(msg.y)
        cam_point.point.z = float(msg.z)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            base_point = do_transform_point(cam_point, tf)
        except TransformException as exc:
            self.get_logger().warn(f"⚠️ TF 변환 실패(camera->base_link): {exc}")
            return

        bx_raw = float(base_point.point.x)
        by_raw = float(base_point.point.y)
        bz_raw = float(base_point.point.z)
        bx = bx_raw + self.base_pick_offset_x
        by = by_raw + self.base_pick_offset_y
        bz = bz_raw + self.base_pick_offset_z
        self.get_logger().info(
            f"📥 좌표 수신됨(cam raw): X={msg.x:.3f}, Y={msg.y:.3f}, Z={msg.z:.3f} | "
            f"(base raw): X={bx_raw:.3f}, Y={by_raw:.3f}, Z={bz_raw:.3f} | "
            f"(base corrected): X={bx:.3f}, Y={by:.3f}, Z={bz:.3f}"
        )
        
        self.is_busy = True
        # 시퀀스 실행 스레드 시작
        threading.Thread(target=self.execute_sequence_thread, args=(bx, by, bz)).start()

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
        self.get_logger().info(f"🛰️ 이동 명령 전송 중: [{x:.3f}, {y:.3f}, {z:.3f}]")

        # OpenMANIPULATOR-X 작업 반경(대략치) 밖 요청은 조기 차단
        radius_xy = math.sqrt(x * x + y * y)
        if radius_xy < 0.10 or radius_xy > 0.35:
            self.get_logger().error(
                f"🚫 작업 반경 밖 좌표: r={radius_xy:.3f}m (x={x:.3f}, y={y:.3f})"
            )
            return False
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 3.0 # 계획 시간 3초
        goal_msg.request.num_planning_attempts = 10
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        # 엔드이펙터가 바닥을 바라보도록 설정 (수정됨)
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.707
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.707
        
        constraints = Constraints()
        p_con = PositionConstraint()
        p_con.header.frame_id = "base_link"
        p_con.link_name = "end_effector_link"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.03, 0.03, 0.03] # 허용 오차 3cm
        p_con.constraint_region.primitives.append(box)
        p_con.constraint_region.primitive_poses.append(target_pose)
        p_con.weight = 1.0
        constraints.position_constraints.append(p_con)

        # Keep end-effector facing downward for stable top-down grasping.
        o_con = OrientationConstraint()
        o_con.header.frame_id = "base_link"
        o_con.link_name = "end_effector_link"
        o_con.orientation = target_pose.orientation
        o_con.absolute_x_axis_tolerance = 0.15
        o_con.absolute_y_axis_tolerance = 0.15
        o_con.absolute_z_axis_tolerance = 0.25
        o_con.weight = 1.0
        constraints.orientation_constraints.append(o_con)

        goal_msg.request.goal_constraints.append(constraints)

        future = self._arm_action_client.send_goal_async(goal_msg)
        
        # 1. 서버 수락 여부 확인
        while rclpy.ok() and not future.done(): time.sleep(0.05)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"🚫 [계획 실패] {x:.2f}, {y:.2f}, {z:.2f} 위치는 로봇이 닿을 수 없습니다.")
            return False

        self.get_logger().info("⚙️ 경로 계획 성공! 로봇 이동 중...")
        
        # 2. 이동 완료 여부 확인
        res_future = goal_handle.get_result_async()
        while rclpy.ok() and not res_future.done(): time.sleep(0.05)

        result_msg = res_future.result()
        if result_msg is None or result_msg.result is None:
            self.get_logger().error("❌ MoveIt 결과 수신 실패")
            return False

        error_val = result_msg.result.error_code.val
        if error_val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"❌ MoveIt 실행 실패 (error_code={error_val})")
            return False

        self.get_logger().info("🏁 목표 지점 도착 완료.")
        return True

    def send_gripper_blocking(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        self._gripper_action_client.send_goal_async(goal_msg)
        time.sleep(1.0) # 그리퍼 동작 대기
        return True

    def execute_sequence_thread(self, tx, ty, tz):
        try:
            self.get_logger().info("🔥 [SEQUENCE] 작업을 시작합니다.")
            
            # 1. 높이 설정 (안전 높이 및 잡기 높이)
            # 비전 z를 신뢰하되, 로봇/테이블 환경을 고려해 안전 범위로 클램프
            PICK_Z = max(0.03, min(tz, 0.12))
            APPROACH_Z = min(PICK_Z + 0.08, 0.20)
            self.get_logger().info(
                f"📏 사용 높이: tz={tz:.3f} -> pick_z={PICK_Z:.3f}, approach_z={APPROACH_Z:.3f}"
            )

            # 그리퍼 열기
            self.send_gripper_blocking(0.019) 
            
            # 2. 물체 위로 접근
            if self.send_precise_goal_blocking(tx, ty, APPROACH_Z):
                # 3. 물체 잡으러 내려가기
                if self.send_precise_goal_blocking(tx, ty, PICK_Z):
                    self.get_logger().info("✊ 물체를 잡습니다.")
                    self.send_gripper_blocking(-0.01) # 꽉 닫기
                    time.sleep(1.0)
                    # 4. 다시 들어올리기
                    self.send_precise_goal_blocking(tx, ty, APPROACH_Z)
            else:
                self.get_logger().warn("⚠️ 첫 번째 이동 시도 실패로 시퀀스를 중단합니다.")
                return

            # 5. 분류 장소로 이동 (조인트 방식 - 더 확실함)
            self.get_logger().info("🚚 분류 장소로 이동 중...")
            self.send_arm_joint_topic([-117.0, -50.0, 58.0, 59.0])
            time.sleep(4.0)
            
            # 6. 물체 놓기
            self.send_gripper_blocking(0.019)
            time.sleep(1.0)
            
            # 7. 홈으로 복귀
            self.get_logger().info("🏠 홈 자세로 복귀합니다.")
            self.send_arm_joint_topic([0.0, -21.0, -8.0, 112.0])
            # 인식 안정화를 위해 홈 자세에서 5초 대기
            time.sleep(5.0)

        except Exception as e:
            self.get_logger().error(f"‼️ 시퀀스 실행 중 오류 발생: {e}")
        finally:
            self.is_busy = False
            self.get_logger().info("✨ [FINISH] 작업 대기 상태로 전환.")

def main():
    rclpy.init()
    node = MasterNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
