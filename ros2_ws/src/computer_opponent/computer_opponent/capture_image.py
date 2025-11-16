#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import time
import threading
from move_detection import MoveDetector
import chess


class CropCaptureNode(Node):
    def __init__(self):
        super().__init__("crop_capture_node")
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )
        self.state_sub = self.create_subscription(
            String, "set_state", self.state_callback_, 10
        )
        self.move_sub = self.create_subscription(
            String, "move_made", self.update_board_callback_, 10
        )
        self.move_pub = self.create_publisher(String, "move_made", 10)

        self.bridge = CvBridge()
        self.timer = None
        self.counter = 0
        self.last_frame = None
        self.move_detector = MoveDetector()
        self.state = "idle"  # calibrated, playing
        self.is_your_turn = True
        # Output folder
        os.makedirs(self.output_dir, exist_ok=True)
        threading.Thread(target=self.read_input_loop, daemon=True).start()

    def image_callback(self, msg):
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def timer_callback_(self):
        if self.last_frame is None:
            return
        img = self.crop_last_image()
        self.save_image(img)
        if not self.is_your_turn:
            return
        move = self.move_detector.detect_move(img)
        if move != None:
            msg = String()
            msg.data = move
            self.move_pub.publish(msg)

    def state_callback_(self, msg):
        if self.last_frame == None:
            self.get_logger().error(
                "Cant change state when image is none, is camera running?"
            )
        if msg.data == "calibrate":
            self.move_detector.find_transform(self.last_frame)
            self.state = "calibrated"
        if msg.data == "start_game":
            if self.move_detector.trsf_matrix == None:
                self.get_logger().error(
                    "Cant start game without transform, run calibrate first"
                )
            self.state = "playing"
            self.timer = self.create_timer(0.5, self.timer_callback_)
            self.get_logger().info("\033[1;32m Game started! \033[0m")

    def update_board_callback_(self, msg):
        try:
            move = chess.Move.from_uci(msg.data)
        except ValueError:
            self.get_logger().info("Received not valid move format")
            return

        if move not in self.board.legal_moves:
            self.get_logger().info("Received illegal move")
            return
        else:
            self.get_logger().info("Received legal move")
        self.move_detector.board.push(move)
        self.is_your_turn = not self.is_your_turn

    def crop_last_image(self):
        # ROI parameters (adjust as needed)
        x_offset = 300  # top-left corner X
        y_offset = 0  # top-left corner Y
        width = 800  # width of crop
        height = 600  # height of crop
        roi = self.last_frame[y_offset : y_offset + height, x_offset : x_offset + width]
        return roi

    def save_image(self, img):
        output_dir = "/home/igorsiata/ur3_chess_player/vision_system/img/game"
        filename = os.path.join(output_dir, f"move{self.counter}.png")
        cv2.imwrite(filename, img)
        self.get_logger().info(f"Saved image: {filename}")
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = CropCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
