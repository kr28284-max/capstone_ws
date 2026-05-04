import math
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        self.callback_group = ReentrantCallbackGroup()
        self.is_busy = True

        # Vision node3가 base_link 기준 좌표를 발행한다고 가정.
        self.target_topic = '/aruco_target_point'

        # 집기 시퀀스 파라미터
        self.approach_height = 0.15
        self.ee_link_name = 'end_effector_link'
        self.goal_frame = 'base_link'
        # 비전 좌표 미세 보정 (m): 현재 조합(vision_master_node2 + test_node1)에서
        # z 대신 x축으로 살짝 치우치는 현상을 보정.
        self.pick_offset_x = -0.015
        self.pick_offset_y = 0.0
        self.pick_offset_z = 0.0

        # 분류/드롭 위치(조인트 이동)
        self.drop_joint_deg = [-117.0, -50.0, 58.0, 59.0]
        self.home_joint_deg = [0.0, -21.0, -8.0, 112.0]

        # ROS 인터페이스
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.arm_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.callback_group)
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd', callback_group=self.callback_group
        )
        self.target_sub = self.create_subscription(
            Point, self.target_topic, self.target_callback, 10, callback_group=self.callback_group
        )

        self.get_logger().info('🚀 [START] master_node 시작. 액션 서버 연결 및 홈 포즈 초기화 진행')
        threading.Thread(target=self.init_robot_sequence, daemon=True).start()

    def init_robot_sequence(self):
        self.get_logger().info('⏳ MoveIt/gripper 서버 대기 중...')

        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ /move_action 서버 연결 실패')
            return

        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ /gripper_controller/gripper_cmd 서버 연결 실패')
            return

        self.get_logger().info('✅ 서버 연결 완료. 홈 포즈로 이동')
        self.send_arm_joint_topic(self.home_joint_deg)
        self.send_gripper_blocking(-0.01)
        time.sleep(3.5)

        self.is_busy = False
        self.get_logger().info('🎯 [READY] 비전 타겟 수신 대기')

    def target_callback(self, msg: Point):
        if self.is_busy:
            return

        self.is_busy = True
        corrected_x = msg.x + self.pick_offset_x
        corrected_y = msg.y + self.pick_offset_y
        corrected_z = msg.z + self.pick_offset_z
        self.get_logger().info(
            f'📥 타겟 수신(base_link): raw=({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) '
            f'-> corrected=({corrected_x:.3f}, {corrected_y:.3f}, {corrected_z:.3f})'
        )
        threading.Thread(
            target=self.execute_sequence_thread,
            args=(corrected_x, corrected_y, corrected_z),
            daemon=True,
        ).start()

    def send_arm_joint_topic(self, joint_degrees):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in joint_degrees]
        point.time_from_start.sec = 2
        msg.points.append(point)

        self.arm_pub.publish(msg)

    def send_precise_goal_blocking(self, x: float, y: float, z: float) -> bool:
        self.get_logger().info(f'🛰️ MoveIt 목표 전송: [{x:.3f}, {y:.3f}, {z:.3f}] ({self.goal_frame})')

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 3.0
        goal_msg.request.num_planning_attempts = 10

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        # 그리퍼가 아래로 향하도록 고정
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
        box.dimensions = [0.01, 0.01, 0.01]

        p_con.constraint_region.primitives.append(box)
        p_con.constraint_region.primitive_poses.append(target_pose)
        p_con.weight = 1.0
        constraints.position_constraints.append(p_con)
        goal_msg.request.goal_constraints.append(constraints)

        send_future = self.arm_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('🚫 MoveIt goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.05)

        self.get_logger().info('🏁 목표 도착 완료')
        return True

    def send_gripper_blocking(self, position: float) -> bool:
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        self.gripper_client.send_goal_async(goal_msg)
        time.sleep(1.0)
        return True

    def execute_sequence_thread(self, tx: float, ty: float, tz: float):
        try:
            self.get_logger().info('🔥 [SEQUENCE] 픽업 시퀀스 시작')

            pick_x = tx
            pick_y = ty
            pick_z = tz
            approach_z = pick_z + self.approach_height

            self.get_logger().info(
                f'🎯 사용 좌표(base_link): pick=({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f}), '
                f'approach_z={approach_z:.3f}'
            )

            self.send_gripper_blocking(0.019)

            if self.send_precise_goal_blocking(pick_x, pick_y, approach_z):
                if self.send_precise_goal_blocking(pick_x, pick_y, pick_z):
                    self.get_logger().info('✊ 그리퍼 닫기')
                    self.send_gripper_blocking(-0.01)
                    time.sleep(1.0)
                    self.send_precise_goal_blocking(pick_x, pick_y, approach_z)
                else:
                    self.get_logger().warn('⚠️ pick 단계 실패')
                    return
            else:
                self.get_logger().warn('⚠️ approach 단계 실패')
                return

            self.get_logger().info('🚚 드롭 위치로 이동')
            self.send_arm_joint_topic(self.drop_joint_deg)
            time.sleep(4.0)

            self.send_gripper_blocking(0.019)
            time.sleep(1.0)

            self.get_logger().info('🏠 홈 포즈 복귀')
            self.send_arm_joint_topic(self.home_joint_deg)
            time.sleep(3.0)

        except Exception as exc:
            self.get_logger().error(f'‼️ 시퀀스 오류: {exc}')
        finally:
            self.is_busy = False
            self.get_logger().info('✨ [FINISH] 다음 타겟 대기')


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
