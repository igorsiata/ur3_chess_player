#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import time
import threading

class CropCaptureNode(Node):
    def __init__(self):
        super().__init__('crop_capture_node')

        # ROS 2 subscription
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.timer = self.create_timer(0.5, self.timer_callback_)
        # ROI parameters (adjust as needed)
        self.x_offset = 300  # top-left corner X
        self.y_offset = 0  # top-left corner Y
        self.width = 800     # width of crop
        self.height = 600    # height of crop
        self.last_frame = None
        self.capture = False
        self.counter = 0
        # Output folder
        self.output_dir = "/home/igorsiata/ur3_chess_player/vision_system/img/game"
        os.makedirs(self.output_dir, exist_ok=True)
        threading.Thread(target=self.read_input_loop, daemon=True).start()

    def timer_callback_(self):
        if self.last_frame is None:
            return
        filename = os.path.join(self.output_dir, f"move{self.counter}.png")
        roi = self.last_frame[self.y_offset:self.y_offset+self.height,
                    self.x_offset:self.x_offset+self.width]
        cv2.imwrite(filename, roi)
        self.get_logger().info(f"Saved image: {filename}")
        self.counter += 1


    def image_callback(self, msg):
        self.last_frame  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        

    def read_input_loop(self):
        while rclpy.ok():
            user_input = input("Press any key to capture image")
            self.capture = True
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = CropCaptureNode()
    rclpy.spin(node)
    # node.destroy_node()
    

if __name__ == '__main__':
    main()
