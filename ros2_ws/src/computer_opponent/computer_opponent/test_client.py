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


    def send_waypoints(self, waypoints):
        squares = ["open", "close", "open", "close"]
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
            # time.sleep(2)

                    

def main(args=None):
    rclpy.init(args=args)
    client = TestClient()

    # Example: define some waypoints
    waypoints = []

    p1 = Pose()
    p1.position.x = 0.3
    p1.position.y = 0.0
    p1.position.z = 0.5
    p1.orientation.w = 1.0
    waypoints.append(p1)

    p2 = Pose()
    p2.position.x = 0.4
    p2.position.y = 0.1
    p2.position.z = 0.5
    p2.orientation.w = 1.0
    waypoints.append(p2)

    p3 = Pose()
    p3.position.x = 0.5
    p3.position.y = 0.0
    p3.position.z = 0.5
    p3.orientation.w = 1.0
    waypoints.append(p3)

    # Send waypoints sequentially
    client.send_waypoints(waypoints)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
