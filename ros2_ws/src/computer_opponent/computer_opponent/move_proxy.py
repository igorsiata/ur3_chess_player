#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ur3_tcp.srv import MoveWaypoint, MoveNamedPose, MakeMove, GripperCmd, MoveJoints

IDLE_POS = [1.57, -1.57, 0.79, -1.57, -1.57, 0.0]
OUT_POS = [2.27, -1.36, 1.94, -2.15, -1.57, 0.7]


class MoveMaker(Node):
    def __init__(self):
        super().__init__("move_maker")
        self.cli_move_waypoint = self.create_client(MoveWaypoint, "move_waypoint")
        self.cli_move_joints = self.create_client(MoveJoints, "move_waypoint")
        self.srv = self.create_service(MakeMove, "make_move", self.make_move_callback)
        self.cli_gripper = self.create_client(GripperCmd, "gripper_cmd")
        self.req_waypoint = MoveJoints.Request()
        self.req_joints = MoveNamedPose.Request()
        self.req_gripper = GripperCmd.Request()

    def make_move_callback(self, request, response):
        if request.move_type == "regular":
            result = self.make_regular_move(request.from_sqr, request.to_sqr)
        if request.move_type == "castle":
            rank = request.to_sqr[1]
            is_kingside = request.to_sqr[0] == "g"
            if is_kingside:
                from_sqr = "h" + rank
                to_sqr = "f" + rank
            else:
                from_sqr = "a" + rank
                to_sqr = "d" + rank
            result = self.make_regular_move(from_sqr, to_sqr) # rook_move
            result = self.make_regular_move(request.from_sqr, request.to_sqr) # king_move
        if request.move_type == "capture":
            result = self.make_capture(request.to_sqr)
            result = self.make_regular_move(request.from_sqr, request.to_sqr)
        if request.move_type == "en_passant":
            result = self.make_capture(request.to_sqr)
            result = self.make_regular_move(request.from_sqr, request.to_sqr)
        if request.move_type == "promotion":
            pass

    def send_gripper_cmd(self, cmd):
        self.req_gripper.cmd = cmd
        future = self.cli_gripper.call(self.req_gripper)
        return future.result()


    def make_regular_move(self, from_square, to_square):
        self.req_waypoint.waypoint = self.square_to_pose(from_square)
        future = self.cli_move_waypoint.call_async(self.req_waypoint)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.sucess:
            return False
        
        # CLOSE GRIPPER

        self.req_waypoint.waypoint = self.square_to_pose(to_square)
        future = self.cli_move_waypoint.call_async(self.req_waypoint)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.sucess:
            return False
        
        # OPEN GRIPPER

        self.req_joints.angles = IDLE_POS
        future = self.cli_move_joints.call_async(self.req_joints)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.success
    
    def make_capture(self, captured_square):
        self.req_waypoint.waypoint = self.square_to_pose(captured_square)
        future = self.cli_move_waypoint.call_async(self.req_waypoint)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.sucess:
            return False
        # go to trash

        # CLOSE GRIPPER

        self.req_joints.angles = OUT_POS
        future = self.cli_move_joints.call_async(self.req_joints)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.sucess:
            return False
        # go idle

        # OPEN GRIPPER
        
        self.req_joints.angles = IDLE_POS
        future = self.cli_move_joints.call_async(self.req_joints)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.success

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
        board_height = 0.0024

        pose = Pose()
        row = int(square[1]) - 1
        col = ord(square[0]) - 97

        a1_cords = chsb_mid[:]
        a1_cords[0] -= sqr_dim[0] * 3.5
        a1_cords[1] -= sqr_dim[1] * 3.5

        pose.position.x = a1_cords[0] + col * sqr_dim[0]
        pose.position.y = a1_cords[1] + row * sqr_dim[1]
        pose.position.z = board_height + pieces_heights_m["b"] + 0.14
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose

    def send_waypoints(self, waypoints):
        squares = ["a1", "a8", "h1", "h8"]
        for sqr in squares:
            self.get_logger().info(f"Sending square {sqr}")

            self.req.waypoint = self.square_to_pose(sqr)
            future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response.success:
                self.get_logger().info(f"Waypoint executed successfully")
            else:
                self.get_logger().error(
                    f"Failed to execute waypoint : {response.message}"
                )


def main(args=None):
    rclpy.init(args=args)
    client = MoveMaker()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
