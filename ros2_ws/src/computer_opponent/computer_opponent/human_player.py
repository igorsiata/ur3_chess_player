import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class ConsoleInputNode(Node):
    def __init__(self):
        super().__init__('console_input_node')
        self.publisher_ = self.create_publisher(String, 'human_move', 10)
        # Start a thread for console input
        threading.Thread(target=self.read_input_loop, daemon=True).start()

    def read_input_loop(self):
        while rclpy.ok():
            user_input = input("Enter something: ")  # Blocking call
            msg = String()
            msg.data = user_input
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {user_input}")
            time.sleep(2)

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

if __name__ == '__main__':
    main()
