import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler


import chess
import chess.engine



class StockfishPlayer(Node):

    def __init__(self):
        super().__init__('stockfish_player')
        
        self.subscriber_ = self.create_subscription(String, 'human_move', self.human_move_callback, 10)
        self.pose_publisher = self.create_publisher(Pose, 'robot_ee_pos', 10)
        # timer_period = 2  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
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
        board_height_m = 0.04
        pieces_heights_m = {
            'p': 0.06,
            'k': 0.08,
            'b': 0.08,
            'r': 0.08,
            'q': 0.08,
            'k': 0.08,
        }
        tile_size_m = 0.055
        board_size_m = 0.540
        board_middle_x = 0.4
        board_middle_y = 0.0

        from_square = self.convert_string_pos_to_xy(move_uci[:2])
        to_square = self.convert_string_pos_to_xy(move_uci[2:4])

        # if move.promotion:
        #     pass
        # if move.is_castling():
        #     pass
        # if move.is_capture():
        #     pass

        msg = self.create_pose(0.1, 0.1, 0.38, 3.14, 0, -1.54)
        self.pose_publisher.publish(msg)


    def create_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> Pose:
        """
        Create a geometry_msgs.msg.Pose message from position (x, y, z)
        and orientation given in roll, pitch, yaw (in radians).

        Args:
            x (float): X position
            y (float): Y position
            z (float): Z position
            roll (float): Roll angle (radians)
            pitch (float): Pitch angle (radians)
            yaw (float): Yaw angle (radians)

        Returns:
            Pose: The resulting Pose message
        """
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)

        # Convert roll, pitch, yaw to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        return pose

        

    def convert_string_pos_to_xy(self, string_pos):
        x = ord(string_pos[1]) - 48
        y = ord(string_pos[0]) - 96
        return x, y


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

