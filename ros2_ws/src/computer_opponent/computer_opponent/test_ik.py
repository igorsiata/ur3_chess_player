#!/usr/bin/env python3
# file: ur3_pose_action_server.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

import numpy as np


# --- UR3 DH parameters (meters) ---
d1 = 0.1519
a2 = -0.24365
a3 = -0.21325
d4 = 0.11235
d5 = 0.08535
d6 = 0.0819


def ur3_inverse_kinematics(px, py, pz, R):
    """
    Compute one analytical IK solution for UR3.
    Returns np.array([q1..q6]) in radians or None if unreachable.
    """
    try:
        R_tool_correction = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])

        R = R @ R_tool_correction
        # Shoulder (q1)
        q1 = np.arctan2(py, px)
        wrist_center = np.array([px, py, pz]) - d6 * R[:, 2]
        xc, yc, zc = wrist_center

        r = np.sqrt(xc**2 + yc**2)
        s = zc - d1
        D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)

        if abs(D) > 1.0:
            return None

        q3 = np.arctan2(-np.sqrt(1 - D**2), D)
        q2 = np.arctan2(s, r) - np.arctan2(a3 * np.sin(q3), a2 + a3 * np.cos(q3))

        R03 = np.array([
            [np.cos(q1) * np.cos(q2 + q3), -np.cos(q1) * np.sin(q2 + q3), np.sin(q1)],
            [np.sin(q1) * np.cos(q2 + q3), -np.sin(q1) * np.sin(q2 + q3), -np.cos(q1)],
            [np.sin(q2 + q3), np.cos(q2 + q3), 0]
        ])

        R36 = R03.T @ R
        q5 = np.arccos(np.clip(R36[2, 2], -1.0, 1.0))
        q4 = np.arctan2(R36[1, 2], R36[0, 2])
        q6 = np.arctan2(R36[2, 1], -R36[2, 0])

        return np.array([q1, q2, q3, q4, q5, q6])
    except Exception as e:
        print(f"IK error: {e}")
        return None


class UR3PoseActionServer(Node):
    def __init__(self):
        super().__init__('ur3_pose_action_server')

        self.declare_parameter('action_name', '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('trajectory_time', 2.0)

        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.trajectory_time = self.get_parameter('trajectory_time').get_parameter_value().double_value

        # Action client
        self.action_client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self.get_logger().info(f"Action client created for {self.action_name}")

        # Subscriber
        self.pose_sub = self.create_subscription(PoseStamped, 'target_pose', self.pose_callback, 10)
        self.get_logger().info("Subscribed to /target_pose")

        # Joint names for UR3
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info("Received pose — computing IK...")

        # Convert quaternion -> rotation matrix
        q = msg.pose.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])

        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        q_sol = ur3_inverse_kinematics(px, py, pz, R)

        if q_sol is None:
            self.get_logger().warn("No IK solution found — pose may be unreachable.")
            return

        self.send_trajectory(q_sol)

    def send_trajectory(self, q_target):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server {self.action_name} not available!")
            return

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        point.time_from_start = Duration(sec=int(self.trajectory_time))
        traj.points = [point]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self.get_logger().info("Sending trajectory action goal...")
        send_future = self.action_client.send_goal_async(goal_msg)

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error("Trajectory goal rejected.")
                return
            self.get_logger().info("Trajectory goal accepted. Waiting for result...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)

        def result_callback(fut):
            result = fut.result().result
            self.get_logger().info(f"Trajectory result received: {result.error_code}")

        send_future.add_done_callback(goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = UR3PoseActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
