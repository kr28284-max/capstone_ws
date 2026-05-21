import math
import threading
import time
from typing import List

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ManualPoseNode(Node):
    def __init__(self):
        super().__init__('manual_pose_node')

        self.callback_group = ReentrantCallbackGroup()
        self.goal_frame = 'base_link'
        self.ee_link_name = 'end_effector_link'

        self.home_joint_deg = [0.0, -6.0, -24.0, 114.0]
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.arm_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.callback_group)
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group,
        )

        threading.Thread(target=self.console_loop, daemon=True).start()

    def console_loop(self):
        self.get_logger().info('Waiting for MoveIt and gripper action servers...')
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_action server connection failed')
            rclpy.shutdown()
            return
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('gripper_cmd server connection failed. Pose movement still works.')

        self.print_help()
        while rclpy.ok():
            try:
                text = input('manual_pose> ').strip()
            except (EOFError, KeyboardInterrupt):
                rclpy.shutdown()
                return

            if not text:
                continue
            cmd = text.lower()
            if cmd in ('q', 'quit', 'exit'):
                rclpy.shutdown()
                return
            if cmd in ('h', 'help', '?'):
                self.print_help()
                continue
            if cmd == 'home':
                self.send_arm_joint_topic(self.home_joint_deg)
                self.get_logger().info(f'sent home joints: {self.home_joint_deg}')
                continue
            if cmd == 'open':
                self.send_gripper_blocking(0.019)
                continue
            if cmd == 'close':
                self.send_gripper_blocking(-0.01)
                continue

            parts = text.replace(',', ' ').split()
            if len(parts) != 3:
                self.get_logger().warn('Enter: x y z  (meter), for example: 0.20 0.00 0.10')
                continue

            try:
                x, y, z = [float(p) for p in parts]
            except ValueError:
                self.get_logger().warn('x y z must be numbers in meters.')
                continue

            self.get_logger().info(f'moving to base_link pose: x={x:.3f}, y={y:.3f}, z={z:.3f}')
            ok = self.send_precise_goal_blocking(x, y, z)
            if ok:
                self.get_logger().info('move success')
            else:
                self.get_logger().warn('move failed')

    def print_help(self):
        self.get_logger().info('Type x y z in meters, example: 0.20 0.00 0.10')
        self.get_logger().info('Commands: home, open, close, help, quit')

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
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('gripper_cmd server is not ready')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        future = self.gripper_client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        time.sleep(0.6)
        self.get_logger().info(f'gripper command sent: {position:.3f}')
        return True


def main():
    rclpy.init()
    node = ManualPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
