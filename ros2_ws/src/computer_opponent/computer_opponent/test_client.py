#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ur3_tcp.srv import GripperCmd, MoveNamedPose
import time


class TestClient(Node):
    def __init__(self):
        super().__init__('arm_mover_client')
        self.cli = self.create_client(GripperCmd, 'gripper_cmd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = GripperCmd.Request()


    def send_waypoints(self):
        squares = ["open", "close_b", "open", "close_p", "open"]
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

                    

def main(args=None):
    rclpy.init(args=args)
    client = TestClient()
    client.send_waypoints()

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
