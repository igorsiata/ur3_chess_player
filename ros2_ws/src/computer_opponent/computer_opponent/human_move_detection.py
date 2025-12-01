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
from computer_opponent.move_detection import (
    MoveDetector,
    find_transform_from_corners,
    find_transform,
)
import chess
import torch
from torchvision import transforms
from PIL import Image as PILImage
from computer_opponent.chess_cnn import ChessCNN
import time


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
            String, "/set_state", self.state_callback_, 10
        )
        self.move_sub = self.create_subscription(
            String, enemy_move_topic, self.update_board_callback_, 10
        )
        self.move_pub = self.create_publisher(String, your_move_topic, 10)
        self.img_pub = self.create_publisher(Image, "/move_detected", 10)
        self.img_pub2 = self.create_publisher(Image, "/image_cropped", 10)

        self.img_size = (100 * 8, 100 * 8)
        self.bridge = CvBridge()
        self.timer = None
        self.counter = 0
        self.last_frame = None
        self.curr_frame = None
        self.board = chess.Board(self.get_parameter("start_position_fen").value)
        self.state = "idle"  # calibrated, playing

        self.DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = ChessCNN().to(self.DEVICE)
        self.model.load_state_dict(
            torch.load(
                "/home/igorsiata/ur3_chess_player/vision_system/model_weights.pth",
                map_location=self.DEVICE,
            )
        )
        self.model.eval()

    transform = transforms.Compose(
        [
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5]),
        ]
    )

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.curr_frame = self.crop_img(img)
        msg = self.bridge.cv2_to_imgmsg(self.curr_frame, encoding="bgr8")
        self.img_pub2.publish(msg)

    def timer_callback_(self):
        if not self.is_your_turn:
            return
        if self.curr_frame is None:
            return
        start = time.time()
        self.detect_move()
        end = time.time()
        self.get_logger().info(f"Inference time: {(end-start)*1000:.2f} ms")
        # self.save_image(self.last_frame)

    def classify_square(self, img):

        labels = ["b", ".", "w"]
        img = self.transform(img).unsqueeze(0).to(self.DEVICE)
        with torch.no_grad():
            out = self.model(img)
            cls = out.argmax(1).item()
        return labels[cls]

    def detect_move(self):
        move_discard_thresh = 2000
        img = cv2.warpPerspective(self.curr_frame, self.trsf_matrix, self.img_size)

        if self.curr_frame is None:
            return None, None
        if self.last_frame is None:
            self.last_frame = img
            # self.get_logger().info("last frame is none")
            return None, None

        diff = cv2.absdiff(
            cv2.cvtColor(img, cv2.COLOR_RGB2GRAY),
            cv2.cvtColor(self.last_frame, cv2.COLOR_RGB2GRAY),
        )
        _, diff_thresh = cv2.threshold(diff, 60, 1, cv2.THRESH_BINARY)
        diff_thresh = cv2.erode(diff_thresh, (3, 3), iterations=1)
        move_count = np.sum(diff_thresh)

        self.last_frame = img
        if move_count >= move_discard_thresh:
            # self.get_logger().info("too much movement")

            return None, None

        # Split into 64 squares
        self.save_image(img)
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.img_pub.publish(msg)
        player_pieces = "w" if self.is_white else "b"
        squares = []
        board_str = "\n"
        SQUARE_SIZE = 100
        for row in range(8):
            for col in range(8):
                sq = img[
                    row * SQUARE_SIZE : (row + 1) * SQUARE_SIZE,
                    col * SQUARE_SIZE : (col + 1) * SQUARE_SIZE,
                ]
                square_pil = PILImage.fromarray(cv2.cvtColor(sq, cv2.COLOR_BGR2RGB))
                label = self.classify_square(square_pil)
                if label == player_pieces:
                    squares.append(row + col * 8)
                board_str += f"{label} "
                if col == 7:
                    board_str += "\n"

        self.get_logger().info(f"Published board: {board_str}")

        if self.is_white:
            chess_color = chess.WHITE
            piece_label = 1
        else:
            chess_color = chess.BLACK
            piece_label = 2

        white_squares = (
            list(self.board.pieces(chess.PAWN, chess_color))
            + list(self.board.pieces(chess.KNIGHT, chess_color))
            + list(self.board.pieces(chess.BISHOP, chess_color))
            + list(self.board.pieces(chess.ROOK, chess_color))
            + list(self.board.pieces(chess.QUEEN, chess_color))
            + list(self.board.pieces(chess.KING, chess_color))
        )

        # self.get_logger().info(f"Published board: {white_squares}")
        # self.get_logger().info(f"BOARD: \n{self.board}")
        moved_piece = list(set(white_squares) - set(squares))
        dest = list(set(squares) - set(white_squares))
        move_str = None
        self.get_logger().info(f"MOVED: {moved_piece}, {dest}")
        # normal move, capture, enpassant
        if len(moved_piece) == 1 and len(dest) == 1:
            start_sq = moved_piece[0]
            end_sq = dest[0]
            move_str = self.square64_to_str(start_sq) + self.square64_to_str(end_sq)

            # Check if the piece is a pawn
            moving_piece_type = self.board.piece_type_at(start_sq)
            if moving_piece_type == chess.PAWN:
                start_rank = start_sq // 8
                end_rank = end_sq // 8
                if self.is_white and start_rank == 6 and end_rank == 7:
                    move_str += "q"
                elif not self.is_white and start_rank == 1 and end_rank == 0:
                    move_str += "q"
        if len(moved_piece) == 2 and len(dest) == 2:
            castles_lookup = [
                (0, 4),
                (4, 7),
                (56, 60),
                (60, 63),
            ]  # rook & king initial positions
            dest_castles = [
                (2, 3),
                (5, 6),
                (58, 59),
                (61, 62),
            ]  # rook & king final positions
            castles_notation = ["e1c1", "e1g1", "e8c8", "e8g8"]  # chess notation

            moved_piece_sorted = tuple(sorted(moved_piece))
            dest_sorted = tuple(sorted(dest))
            self.get_logger().info(
                f"Two pieces moved: {moved_piece_sorted}, {dest_sorted}"
            )

            if moved_piece_sorted in castles_lookup and dest_sorted in dest_castles:
                idx = castles_lookup.index(moved_piece_sorted)
                if idx == dest_castles.index(dest_sorted):
                    move_str = castles_notation[idx]

        if not move_str:
            return None
        self.get_logger().info(f"Piece moved {move_str}")
        try:
            move = chess.Move.from_uci(move_str)
        except ValueError:
            self.get_logger().info("Received not valid move format")
            return None
        if move not in self.board.legal_moves:
            self.get_logger().info("Received illegal move")
        else:
            self.get_logger().info("Received legal move")
            self.board.push(move)
            self.get_logger().info(f"BOARD: \n{self.board}")
            msg = String()
            msg.data = move_str
            self.move_pub.publish(msg)
            self.is_your_turn = False

    def square64_to_str(self, sqr64):
        col = int(sqr64 % 8)
        row = int(sqr64 // 8)

        col_letter = chr(ord("a") + col)
        row_number = str(row + 1)

        return f"{col_letter}{row_number}"

    def state_callback_(self, msg):
        # if not self.last_frame:
        #     self.get_logger().error(
        #         "Cant change state when image is none, is camera running?"
        #     )
        if msg.data == "calibrate":
            # self.trsf_matrix = find_transform(self.curr_frame)
            corners = [
                [868.0, 619.0],
                [315.0, 638.0],
                [847.0, 61.0],
                [292.0, 81.0],
            ]
            self.trsf_matrix = self.find_transform_from_corners(corners)
            self.state = "calibrated"
            self.get_logger().info("Calibrated")
        if msg.data == "start_game":
            # self.save_image(self.last_frame)
            # corners = [[977.0, 807.0], [398.0, 804.0], [960.0, 235.0], [398.0, 245.0]]
            # self.trsf_matrix = self.find_transform_from_corners(corners)
            if self.trsf_matrix is None:
                self.get_logger().error(
                    "Cant start game without transform, run calibrate first"
                )
            self.state = "playing"
            self.is_your_turn = self.get_parameter("is_white").value
            self.timer = self.create_timer(1.0, self.timer_callback_)
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
        self.board.push(move)
        self.is_your_turn = True
        self.last_stable_frame = None

    def crop_img(self, img):
        # ROI parameters (adjust as needed)
        x_offset = 500  # top-left corner X
        y_offset = 250  # top-left corner Y
        width = 1920 - 500 - 400  # width of crop
        height = 1080 - 150  # height of crop
        roi = img[y_offset : y_offset + height, x_offset : x_offset + width]
        return roi

    def save_image(self, img):
        output_dir = "/home/igorsiata/ur3_chess_player/vision_system/img/game"
        filename = os.path.join(output_dir, f"moveq{self.counter}.png")
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

    def find_transform_from_corners(self, corners):
        def order_points(pts):
            rect = np.zeros((4, 2), dtype="float32")
            s = pts.sum(axis=1)
            rect[0] = pts[np.argmin(s)]  # top-left
            rect[3] = pts[np.argmax(s)]  # bottom-right

            diff = np.diff(pts, axis=1)
            rect[1] = pts[np.argmin(diff)]  # top-right
            rect[2] = pts[np.argmax(diff)]  # bottom-left

            return rect

        width, height = 800, 800
        corners = np.float32(corners)
        corners = order_points(corners)
        pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
        matrix = cv2.getPerspectiveTransform(corners, pts2)
        return matrix


def main(args=None):
    rclpy.init(args=args)
    node = MoveDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
