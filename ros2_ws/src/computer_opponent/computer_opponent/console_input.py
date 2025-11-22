import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import chess


class ConsoleInputNode(Node):
    def __init__(self):
        super().__init__("console_input_node")
        self.declare_parameter("is_white", True)
        self.declare_parameter(
            "start_position_fen",
            "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
        )
        self.is_white = self.get_parameter("is_white").value
        enemy_move_topic = "black_move" if self.is_white else "white_move"
        your_move_topic = "white_move" if self.is_white else "black_move"
        self.state = "idle"
        self.is_your_turn = self.is_white

        self.move_sub = self.create_subscription(
            String, enemy_move_topic, self.update_board_callback_, 10
        )
        self.move_pub = self.create_publisher(String, your_move_topic, 10)

        start_position = self.get_parameter("start_position_fen").value
        self.board = chess.Board(start_position)

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
        self.read_input()


    def read_input(self):
        while True:
            user_input = input("Enter something: ")  # Blocking call
            try:
                move = chess.Move.from_uci(user_input)
            except ValueError:
                self.get_logger().info("Move not valid format")
                continue

            if move not in self.board.legal_moves:
                self.get_logger().info("Move not legal")
                continue
            else:
                self.get_logger().info(f"Sending legal legal move {user_input}")
                msg = String()
                msg.data = user_input
                self.move_pub.publish(msg)
                return
        
        
def main(args=None):
    rclpy.init(args=args)
    node = ConsoleInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
