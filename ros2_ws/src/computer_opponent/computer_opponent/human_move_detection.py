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
import numpy as np
from computer_opponent.move_detection import MoveDetector
import chess


class MoveDetectionNode(Node):
    def __init__(self):
        super().__init__("move_detection_node")

        self.declare_parameter("is_white", True)
        self.declare_parameter(
            "start_position_fen",
            "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
        )
        self.is_white = self.get_parameter("is_white").value
        enemy_move_topic = "black_move" if self.is_white else "white_move"
        your_move_topic = "white_move" if self.is_white else "black_move"

        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )
        self.state_sub = self.create_subscription(
            String, "set_state", self.state_callback_, 10
        )
        self.move_sub = self.create_subscription(
            String, enemy_move_topic, self.update_board_callback_, 10
        )
        self.move_pub = self.create_publisher(String, your_move_topic, 10)
        self.img_pub = self.create_publisher(Image, "/move_detected", 10)

        self.bridge = CvBridge()
        self.timer = None
        self.counter = 0
        self.last_frame = None
        self.move_detector = MoveDetector(
            self.get_parameter("start_position_fen").value
        )
        self.state = "idle"  # calibrated, playing

        # Output folder
        # os.makedirs(self.output_dir, exist_ok=True)
        # threading.Thread(target=self.read_input_loop, daemon=True).start()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.last_frame = img

    def timer_callback_(self):
        if not self.is_your_turn:
            return
        if self.last_frame is None:
            return
        # img = self.crop_last_image()
        # self.save_image(img)
        if not self.is_your_turn:
            return
        move, img = self.move_detector.detect_move(self.last_frame)
        if move is not None:
            msg = String()
            msg.data = move
            self.move_pub.publish(msg)
            self.is_your_turn = False
            self.get_logger().info(f"Detected move: {move}")
            self.get_logger().info(f"BOARD:\n{self.move_detector.board}")
        if img is not None:
            self.publish_image(img)

    def state_callback_(self, msg):
        # if not self.last_frame:
        #     self.get_logger().error(
        #         "Cant change state when image is none, is camera running?"
        #     )
        if msg.data == "calibrate":
            corners = [[502.0, 358.0], [246.0, 360.0], [505.0, 107.0], [249.0, 103.0]]
            self.move_detector.find_transform_from_corners(self.last_frame, corners)
            self.state = "calibrated"
            self.get_logger().info("Calibrated")
        if msg.data == "start_game":
            self.save_image(self.last_frame)
            corners = [[938.0, 555.0], [514.0, 555.0], [946.0, 138.0], [518.0, 133.0]]
            self.move_detector.find_transform_from_corners(self.last_frame, corners)
            self.state = "calibrated"
            self.get_logger().info("Calibrated")
            if self.move_detector.trsf_matrix is None:
                self.get_logger().error(
                    "Cant start game without transform, run calibrate first"
                )
            self.state = "playing"
            self.is_your_turn = self.get_parameter("is_white").value
            self.timer = self.create_timer(0.5, self.timer_callback_)
            self.get_logger().info("\033[1;32m Game started! \033[0m")

    def update_board_callback_(self, msg):
        try:
            move = chess.Move.from_uci(msg.data)
        except ValueError:
            self.get_logger().info("Received not valid move format")
            return

        if move not in self.move_detector.board.legal_moves:
            self.get_logger().info("Received illegal move")
            return
        else:
            self.get_logger().info("Received legal move")
        self.move_detector.board.push(move)
        self.is_your_turn = True
        self.move_detector.last_stable_frame = None

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

    def publish_image(self, img):
        if img is None:
            return

        if img.dtype != np.uint8:
            img = (img * 255).astype(np.uint8)

        # Jeśli grayscale (1 kanał)
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.img_pub.publish(msg)
        # self.get_logger().info('Image published!')


def main(args=None):
    rclpy.init(args=args)
    node = MoveDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
