#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ur3_tcp.srv import GripperCmd, MoveNamedPose
from ur3_tcp.msg import RobotMoveStatus, MoveDetectionStatus
import time


class TestClient(Node):
    def __init__(self):
        super().__init__('arm_mover_client')
        self.cli = self.create_client(GripperCmd, 'gripper_cmd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = GripperCmd.Request()
        self.pub_robot_state = self.create_publisher(RobotMoveStatus, "robot_move_status", 10)
        self.pub_detection_state = self.create_publisher(MoveDetectionStatus, "move_detection_status", 10)


    def send_waypoints(self):
        squares = ["open"]
        for sqr in squares:
            self.get_logger().info(f'Sending square {sqr}')
            self.req.action =  sqr
            future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response.success:
                self.get_logger().info(f'Waypoint executed successfully')
            else:
                self.get_logger().error(f'Failed to execute waypoint : {response.message}')

    def send_robot_status(self):
        msg = RobotMoveStatus()
        msg.success = False
        msg.move = "a1a2"
        msg.error_message = "Failed :("
        self.pub_robot_state.publish(msg)
        time.sleep(2.0)
        self.get_logger().info('Sent message')
        msg = RobotMoveStatus()
        msg.success = False
        msg.move = "e2e4"
        msg.error_message = "Failed badly"
        self.pub_robot_state.publish(msg)
        time.sleep(2.0)
        self.get_logger().info('Sent message')
        msg = RobotMoveStatus()
        msg.success = True
        msg.move = "e2e4"
        msg.error_message = "not failed "
        self.pub_robot_state.publish(msg)

    def send_move_detection_status(self):
        msg = MoveDetectionStatus()
        msg.legal = True
        msg.move = "calibrated"
        self.pub_detection_state.publish(msg)
        time.sleep(2.0)
        msg = MoveDetectionStatus()
        msg.legal = True
        msg.move = "a1a2"
        msg.changed_squares = []
        self.pub_detection_state.publish(msg)
        time.sleep(2.0)
        self.get_logger().info('Sent message')
        msg = MoveDetectionStatus()
        msg.legal = False
        msg.move = ""
        msg.changed_squares = ["a1", "a2", "a3"]
        self.pub_detection_state.publish(msg)
        time.sleep(2.0)
        self.get_logger().info('Sent message')
        msg = MoveDetectionStatus()
        msg.legal = False
        msg.move = ""
        msg.changed_squares = []
        self.pub_detection_state.publish(msg)

                    

def main(args=None):
    rclpy.init(args=args)
    client = TestClient()
    client.send_waypoints()
    # client.send_move_detection_status()
    # client.send_robot_status()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
