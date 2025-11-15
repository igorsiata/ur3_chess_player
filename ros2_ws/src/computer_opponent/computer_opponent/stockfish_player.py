import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
from ur3_tcp.srv import MakeMove, GripperCmd
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import chess
import chess.engine



class StockfishPlayer(Node):

    def __init__(self):
        super().__init__('stockfish_player')
        make_move_cb_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_ = self.create_subscription(String, 'human_move', self.human_move_callback, 10)
        self.move_client_ = self.create_client(MakeMove, 'make_move', callback_group=make_move_cb_group)

        engine_path = "/usr/games/stockfish"
        start_position = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        custom_position = "8/8/8/8/8/3R4/2Q5/7k w - - 0 1"
        custom_position_mate_in1 = "rnbqkbnr/pppp1ppp/8/4p3/6P1/5P2/PPPPP2P/RNBQKBNR b KQkq - 0 2"
        self.board = chess.Board(start_position)
        self.engine = chess.engine.SimpleEngine.popen_uci(engine_path)
        self.engine.configure({
            "Skill Level": 5,
            "UCI_LimitStrength": True,
            "UCI_Elo": 1400
        })
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
        self.make_move_on_board(result.move)
        self.get_logger().info(f'Makinng move: {result.move}')
        self.board.push(result.move)
        self.get_logger().info(f'BOARD:\n{self.board}')
        
    def make_move_on_board(self, move: chess.Move):
        move_uci = move.uci()
        
        request = MakeMove.Request()
        request.from_sqr = move_uci[:2]
        request.to_sqr = move_uci[2:4]
        if self.board.is_en_passant(move):
            request.move_type = "enpassant"
        elif self.board.is_capture(move):
            request.move_type = "capture"
        elif self.board.is_castling(move):
            request.move_type = "castle"
        elif len(move_uci) == 5:
            request.move_type = "promotion"
        else:
            request.move_type = "regular"    
        
        self.get_logger().info(f"moving arm: {move_uci}, type {request.move_type}")
        future = self.move_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f'Waypoint executed successfully')
        else:
            self.get_logger().error(f'Failed to execute waypoint : {response.message}')        
        
      


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

