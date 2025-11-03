import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
from ur3_tcp.srv import MakeMove, GripperCmd

import chess
import chess.engine



class StockfishPlayer(Node):

    def __init__(self):
        super().__init__('stockfish_player')
        
        self.subscriber_ = self.create_subscription(String, 'human_move', self.human_move_callback, 10)
        self.move_client_ = self.create_client(MakeMove, 'make_move')
        self.gripper_client_ = self.create_client(GripperCmd, 'gripper_cmd')

        engine_path = "/usr/games/stockfish"
        start_position = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        custom_position = "8/8/8/8/8/3R4/2Q5/7k w - - 0 1"
        custom_position_mate_in1 = "rnbqkbnr/pppp1ppp/8/4p3/6P1/5P2/PPPPP2P/RNBQKBNR b KQkq - 0 2"
        self.board = chess.Board(start_position)
        self.engine = chess.engine.SimpleEngine.popen_uci(engine_path)
        self.print_game_status()

    def human_move_callback(self, msg):
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
        self.print_game_status()
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
        self.get_logger().info(f'BOARD:\n{self.board}')

    def stockfish_make_move(self):
        if self.board.is_game_over():
            self.get_logger().info(f"Game over: {self.board.result()}")
            return
        result = self.engine.play(self.board, chess.engine.Limit(time=0.5))
        self.get_logger().info(f'Makinng move: {result.move}')
        self.board.push(result.move)
        self.get_logger().info(f'BOARD:\n{self.board}')

        self.make_move_on_board(result.move)



    def make_move_on_board(self, move: chess.Move):
        move_uci = move.uci()
      


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = StockfishPlayer()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

