#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from enum import Enum
from ur3_tcp.srv import MoveWaypoint, MoveNamedPose, MakeMove


class MoveType(Enum):
    REGULAR = 0
    CAPTURE = 1
    CASTLE = 2
    ENPASSANT = 3
    PROMOTION = 4
    CAPTURE_PROMOTION = 5


class ArmMoverClient(Node):
    def __init__(self):
        super().__init__("arm_mover_client")
        self.cli = self.create_client(MoveWaypoint, "move_waypoint")
        self.cli_name = self.create_client(MoveNamedPose, "move_named_pose")
        self.cli_move = self.create_client(MakeMove, "make_move")
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')
        self.req = MoveWaypoint.Request()
        self.req_name = MoveNamedPose.Request()
        self.req_move = MakeMove.Request()

    def square_to_pose(
        self,
        square: str,
    ) -> Pose:
        chsb_mid = [0.0, 0.3]
        sqr_dim = [0.0374, 0.0374]
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

    def send_waypoints(self):
        squares = ["a1", "a5", "a2", "a3"]
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

    def send_poses(self):
        poses = [
            "idle",
            "out",
            "idle",
            "out",
        ]
        for pose in poses:
            self.get_logger().info(f"Sending square {pose}")

            self.req_name.name = pose
            future = self.cli_name.call_async(self.req_name)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response.success:
                self.get_logger().info(f"Waypoint executed successfully")
            else:
                self.get_logger().error(
                    f"Failed to execute waypoint : {response.message}"
                )

    def send_moves(self):
        moves = ["e2e4", "h1h8", "e1g1", "b4c3", "a2a1", "d7e8"]
        move_types = [
            MoveType.REGULAR,
            MoveType.CAPTURE,
            MoveType.CASTLE,
            MoveType.ENPASSANT,
            MoveType.PROMOTION,
            MoveType.CAPTURE_PROMOTION
        ]
        for m, mt in zip(moves[3:], move_types[3:]):
            self.get_logger().info(f"Sending move: {mt}, {m}")

            self.req_move.from_sqr = m[:2]
            self.req_move.to_sqr = m[2:]
            self.req_move.move_type = mt.value
            future = self.cli_move.call_async(self.req_move)
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
    client = ArmMoverClient()
    # client.send_waypoints()
    # client.send_poses()
    client.send_moves()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
