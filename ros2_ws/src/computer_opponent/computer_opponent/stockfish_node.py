import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
from ur3_tcp.srv import MakeMove, GripperCmd
from ur3_tcp.msg import RobotMoveStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import chess
import chess.engine
from enum import IntEnum

class MoveType(IntEnum):
    REGULAR = 0
    CAPTURE = 1
    CASTLE = 2
    ENPASSANT = 3
    PROMOTION = 4
    CAPTURE_PROMOTION = 5


class StockfishNode(Node):

    def __init__(self):
        super().__init__("stockfish_node")
        make_move_cb_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("is_white", False)
        self.declare_parameter("skill_level", 5)
        self.declare_parameter("start_position_fen", "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")

        self.is_white = self.get_parameter("is_white").value
        enemy_move_topic = "black_move" if self.is_white else "white_move"
        your_move_topic = "white_move" if self.is_white else "black_move"
        self.state = "idle"

        self.move_client_ = self.create_client(
            MakeMove, "make_move", callback_group=make_move_cb_group
        )
        self.move_subscriber = self.create_subscription(
            String, enemy_move_topic, self.enemy_move_callback_, 10
        )
        self.state_sub = self.create_subscription(
            String, "set_state", self.state_callback_, 10
        )
        self.move_publisher = self.create_publisher(
            String, your_move_topic, 10
        )
        self.robot_move_status_publisher = self.create_publisher(
            RobotMoveStatus, "robot_move_status", 10
        )

        self.configure_engine()

    def configure_engine(self):
        engine_path = "/usr/games/stockfish"
        start_position = self.get_parameter("start_position_fen").value
        self.board = chess.Board(start_position)
        self.engine = chess.engine.SimpleEngine.popen_uci(engine_path)
        self.engine.configure(
            {
                "Skill Level": self.get_parameter("skill_level").value,
                "UCI_LimitStrength": False,
            }
        )

    def state_callback_(self, msg):
        if msg.data == "start_game":
            self.state = "playing"
            self.get_logger().info("\033[1;32m Stocfish engine: game started! \033[0m")
            if self.is_white:
                self.print_game_status()
                self.stockfish_make_move()
                
            
    def enemy_move_callback_(self, msg):
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
        self.stockfish_make_move()
        self.print_game_status()

    def print_game_status(self):
        if self.board.is_checkmate():
            self.get_logger().info("Game over: Checkmate!")
        elif self.board.is_stalemate():
            self.get_logger().info("Game over: Stalemate!")
        elif self.board.is_insufficient_material():
            self.get_logger().info("Game over: Draw by insufficient material!")
        elif self.board.is_fivefold_repetition() or self.board.is_seventyfive_moves():
            self.get_logger().info("Game over: Draw by repetition/75-move rule!")
        elif self.board.is_game_over():
            self.get_logger().info("Game over: Other reason!")
        else:
            self.get_logger().info("Game still in progress.")
            
        self.get_logger().info(f"BOARD:\n{self.board}")

    def stockfish_make_move(self):
        if self.board.is_game_over():
            self.get_logger().info(f"Game over: {self.board.result()}")
            return
        result = self.engine.play(self.board, chess.engine.Limit(time=0.5))
        move = result.move
        self.make_move_on_board(move)
        self.board.push(move)
        self.publish_move(move)

    def always_promote_to_queen(self, move: chess.Move):
        if move.promotion != None:
            return chess.Move(move.from_square, move.to_square, promotion=chess.QUEEN)
        return move

    def publish_move(self, move):
        move_uci = move.uci()
        msg = String()
        msg.data = move_uci
        self.move_publisher.publish(msg)

    def make_move_on_board(self, move: chess.Move):
        move_uci = move.uci()
        request = MakeMove.Request()
        request.from_sqr = move_uci[:2]
        request.to_sqr = move_uci[2:4]

        request.moved_piece = ord(self.board.piece_at(move.from_square).symbol().lower())
        captured_piece = self.board.piece_at(move.from_square)
        if captured_piece is not None:
            request.captured_piece = ord(captured_piece.symbol().lower())

        if self.board.is_en_passant(move):
            request.move_type = MoveType.ENPASSANT
        elif self.board.is_capture(move) and len(move_uci) == 5:
            request.move_type = MoveType.CAPTURE_PROMOTION
        elif self.board.is_capture(move):
            request.move_type = MoveType.CAPTURE
        elif self.board.is_castling(move):
            request.move_type = MoveType.CASTLE
        elif len(move_uci) == 5:
            request.move_type = MoveType.PROMOTION
        else:
            request.move_type = MoveType.REGULAR

        self.get_logger().info(f"moving arm: {move_uci}, type {request.move_type}")
        future = self.move_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f"Waypoint executed successfully")
        else:
            self.get_logger().error(f"Failed to execute waypoint : {response.message}")

        status_msg = RobotMoveStatus()
        status_msg.move = move_uci
        status_msg.success = response.success
        status_msg.error_message = response.error_message
        self.robot_move_status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = StockfishNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
