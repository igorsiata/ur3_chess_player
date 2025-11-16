#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ur3_tcp.srv import MoveWaypoint, MoveNamedPose, MakeMove, GripperCmd, MoveJoints
from rclpy.executors import MultiThreadedExecutor
import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class MoveMaker(Node):
    def __init__(self):
        super().__init__("move_maker")
        move_waypoint_cb_group = MutuallyExclusiveCallbackGroup()
        move_pose_cb_group = MutuallyExclusiveCallbackGroup()
        make_move_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli_move_waypoint = self.create_client(MoveWaypoint, "move_waypoint", callback_group=move_waypoint_cb_group)
        self.cli_move_pose = self.create_client(MoveNamedPose, "move_named_pose", callback_group=move_pose_cb_group)

        if not self.cli_move_waypoint.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service "move_waypoint" not available')
        else:
            self.get_logger().info('Service "move_waypoint" available')
        if not self.cli_move_pose.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service "move_named_pose" not available')
        else:
            self.get_logger().info('Service "move_named_pose" available')

        self.srv = self.create_service(MakeMove, "make_move", self.make_move_callback)
        self.get_logger().info("Move maker started")

    def make_move_callback(self, request, response):
        self.get_logger().info("Received move")
        if request.move_type == "regular":
            self.make_regular_move(request.from_sqr, request.to_sqr)
            self.go_idle()
        if request.move_type == "castle":
            rank = request.to_sqr[1]
            is_kingside = request.to_sqr[0] == "g"
            if is_kingside:
                from_sqr = "h" + rank
                to_sqr = "f" + rank
            else:
                from_sqr = "a" + rank
                to_sqr = "d" + rank
            # rook_move then king_move; both must succeed
            self.make_regular_move(from_sqr, to_sqr)
            self.make_regular_move(request.from_sqr, request.to_sqr)
            self.go_idle()
        if request.move_type == "capture":
            self.make_capture(request.to_sqr)
            self.make_regular_move(request.from_sqr, request.to_sqr)
            self.go_idle()
        if request.move_type == "enpassant":
            self.make_capture(request.to_sqr)
            self.make_regular_move(request.from_sqr, request.to_sqr)
            self.go_idle()
        if request.move_type == "promotion":
            pass
        response.success = True
        self.get_logger().info('Service "make_move" available')
        return response

    def send_gripper_cmd(self, cmd):
        self.req_gripper.cmd = cmd
        future = self.cli_gripper.call(self.req_gripper)
        return future.result()


    def make_regular_move(self, from_square, to_square):
        waypoint_request = MoveWaypoint.Request()
        
        self.get_logger().info(f"making move...")
        waypoint_request.waypoint = self.square_to_pose(from_square)
        future = self.cli_move_waypoint.call_async(waypoint_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f'Waypoint executed successfully')
        else:
            self.get_logger().error(f'Failed to execute waypoint : {response.message}')

        # CLOSE GRIPPER

        waypoint_request.waypoint = self.square_to_pose(to_square)
        future = self.cli_move_waypoint.call_async(waypoint_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f'Waypoint executed successfully')
        else:
            self.get_logger().error(f'Failed to execute waypoint : {response.message}')
        
        # OPEN GRIPPER

    def go_idle(self):
        named_pose_request = MoveNamedPose.Request()
        named_pose_request.name = "idle"
        future = self.cli_move_pose.call_async(named_pose_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f'Waypoint executed successfully')
        else:
            self.get_logger().error(f'Failed to execute waypoint : {response.message}')
        
    
    def make_capture(self, captured_square):
        waypoint_request = MoveWaypoint.Request()
        named_pose_request = MoveNamedPose.Request()
        self.get_logger().info(f"making move capture...")
        if not self.cli_move_waypoint.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('move_waypoint service not available when trying to capture')
            return False

        waypoint_request.waypoint = self.square_to_pose(captured_square)
        future = self.cli_move_waypoint.call_async(waypoint_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.success:
            return False
        # go to trash

        # CLOSE GRIPPER

        named_pose_request.name = "out"
        future = self.cli_move_pose.call_async(named_pose_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.success:
            return False
        
        # OPEN GRIPPER

    def square_to_pose(
        self,
        square: str,
    ) -> Pose:
        chsb_mid = [0.0, 0.3]
        sqr_dim = [0.037, 0.037]
        pieces_heights_m = {
            "p": 0.06,
            "k": 0.08,
            "b": 0.08,
            "r": 0.08,
            "q": 0.08,
            "k": 0.08,
        }

        board_height = 0.024
        gripper_to_tool0_distance = 0.165
        piece_gripping_distance = 0.01
        veritcal_move_offset = 0.06

        pose = Pose()
        row = int(square[1]) - 1
        col = ord(square[0]) - 97

        a1_cords = chsb_mid[:]
        a1_cords[0] += sqr_dim[0] * 3.5
        a1_cords[1] += sqr_dim[1] * 3.5

        pose.position.x = a1_cords[0] - col * sqr_dim[0]
        pose.position.y = a1_cords[1] - row * sqr_dim[1]
        pose.position.z = board_height + piece_gripping_distance + gripper_to_tool0_distance + veritcal_move_offset
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = MoveMaker()
    rclpy.spin(node)
    rclpy.shutdown()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
