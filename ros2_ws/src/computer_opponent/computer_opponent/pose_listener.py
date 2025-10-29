#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# MoveIt2 imports
from moveit2 import MoveIt2
from moveit2 import MoveIt2Interface
from moveit2.ros_planning_interface._moveit2_pybind import PlanningResult

# This assumes you have MoveIt2 properly configured for your robot
# and sourced in your ROS2 workspace environment.

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')

        # Subscribe to Pose topic
        self.subscription = self.create_subscription(
            Pose,
            'Pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint',
                'wrist_2_joint', 'wrist_3_joint'  # modify according to your robot
            ],
            base_link_name='base_link',
            end_effector_name='tool0',
            group_name='ur_arm',  # same as MoveIt group name
        )

        self.get_logger().info('PoseListener with MoveIt2 initialized. Waiting for Pose messages...')

    def listener_callback(self, msg: Pose):
        self.get_logger().info(
            f"Received target Pose: "
            f"Position({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}) | "
            f"Orientation({msg.orientation.x:.3f}, {msg.orientation.y:.3f}, {msg.orientation.z:.3f}, {msg.orientation.w:.3f})"
        )

        # Move to the target pose using MoveIt2
        self.moveit2.move_to_pose(
            position=[msg.position.x, msg.position.y, msg.position.z],
            quat_xyzw=[
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ],
            tolerance_position=0.001,
            tolerance_orientation=0.01,
            max_velocity_scaling_factor=0.5,
            max_acceleration_scaling_factor=0.5
        )

        result = self.moveit2.wait_until_executed()
        if result:
            self.get_logger().info("Motion executed successfully.")
        else:
            self.get_logger().warn("Failed to reach target pose.")


def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
