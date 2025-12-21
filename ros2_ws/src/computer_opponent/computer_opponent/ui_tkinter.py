import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import threading
import sys

from ur3_tcp.msg import RobotMoveStatus, MoveDetectionStatus

# --- 1. Tkinter Application Class ---
class MoveDisplayGUI(tk.Tk):
    """A Tkinter GUI application to display chess moves and current turn."""
    
    def __init__(self, node):
        super().__init__()
        
        self.node = node
        self.title("Gra z robotem")
        self.geometry("600x400")

        self.human_pieces = "WHITE"
        
        # Internal state
        self.current_turn = "WHITE"
        self.move_number = 1 # Tracks the current move number
        self.last_white_move = ""
        self.last_black_move = ""

        # Status variables for the new sub-windows
        self.detection_status_var = tk.StringVar(value="Szachownica nie wykryta")
        self.detection_detail_var = tk.StringVar(value="")
        self.robot_status_indicator_var = tk.StringVar(value="GOTOWY")
        self.robot_status_detail_var = tk.StringVar(value="")

        # --- Configure Grid Layout (2 columns) ---
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=3) # Main Display (Turn)
        self.grid_columnconfigure(1, weight=1) # Side Panel (Last Move)

        # ----------------------------------------------------
        # COLUMN 0: Main Display - Current Turn
        # ----------------------------------------------------
        main_frame = ttk.Frame(self, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")

        # Configure the grid inside main_frame
        main_frame.grid_rowconfigure(0, weight=3) # Turn Indicator gets most space
        main_frame.grid_rowconfigure(1, weight=1) # Move Detection
        main_frame.grid_rowconfigure(2, weight=1) # Robot Move
        main_frame.grid_columnconfigure(0, weight=1)
        
        turn_string = "Twój ruch" if self.current_turn==self.human_pieces else "Ruch robota"
        self.turn_var = tk.StringVar(value=turn_string)
        self.turn_label = ttk.Label(
            main_frame, 
            textvariable=self.turn_var, 
            font=('Arial', 40, 'bold'), 
            anchor='center'
        )
        self.turn_label.grid(row=0, column=0, sticky="nsew")
        self._update_turn_color() # Set initial color

        # 2. Move Detection Status (Row 1)
        detection_frame = ttk.LabelFrame(main_frame, text="Detekcja Ruchu Info", padding="10")
        detection_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        detection_frame.grid_columnconfigure(0, weight=1)
        
        self.detection_label = ttk.Label(
            detection_frame, 
            textvariable=self.detection_status_var, 
            font=('Arial', 14, 'bold'),
            anchor='w'
        )
        self.detection_label.grid(row=0, column=0, sticky="w")

        self.detection_detail = ttk.Label(
            detection_frame, 
            textvariable=self.detection_detail_var, 
            font=('Arial', 11),
            wraplength=350,
            anchor='w'
        )
        self.detection_detail.grid(row=1, column=0, sticky="w")


        # 3. Robot Move Status (Row 2)
        robot_frame = ttk.LabelFrame(main_frame, text="Robot Info", padding="10")
        robot_frame.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)
        robot_frame.grid_columnconfigure(0, weight=1)

        # 3a. Status INDICATOR (Bold, Colored Text)
        self.robot_indicator_label = tk.Label(
            robot_frame,
            textvariable=self.robot_status_indicator_var,
            font=('Arial', 14, 'bold'),  # BOLD FONT
            anchor='w'
        )
        self.robot_indicator_label.grid(row=0, column=0, sticky="w")

        # 3b. Status DESCRIPTION (Detailed text below the indicator)
        self.robot_detail_label = ttk.Label(
            robot_frame, 
            textvariable=self.robot_status_detail_var, 
            font=('Arial', 11),
            wraplength=350, # Help wrap long error messages
            anchor='w'
        )
        self.robot_detail_label.grid(row=1, column=0, sticky="w")

        # 4. Control Buttons (Row 3)
        button_frame = ttk.Frame(main_frame, padding="5")
        button_frame.grid(row=3, column=0, sticky="ew", padx=5, pady=10)
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(1, weight=1)

        # Calibrate Button
        self.calibrate_button = ttk.Button(
            button_frame,
            text="Wykryj szachownicę",
            command=lambda: self.publish_state_command("calibrate") # Command to call
        )
        self.calibrate_button.grid(row=0, column=0, sticky="ew", padx=5)

        # Start Game Button
        self.start_game_button = ttk.Button(
            button_frame,
            text="Rozpocznij grę",
            command=lambda: self.publish_state_command("start_game") # Command to call
        )
        self.start_game_button.grid(row=0, column=1, sticky="ew", padx=5)

        # ----------------------------------------------------
        # COLUMN 1: Side Panel - Scrollable History Log
        # ----------------------------------------------------
        side_frame = ttk.LabelFrame(self, text="Historia gry", padding="10")
        side_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        side_frame.grid_rowconfigure(0, weight=1) # Give all vertical space to the Text widget
        side_frame.grid_columnconfigure(0, weight=1)

        # 1. Scrollbar
        scrollbar = ttk.Scrollbar(side_frame)
        scrollbar.grid(row=0, column=1, sticky="ns")

        # 2. Text Widget for History
        self.history_text = tk.Text(
            side_frame,
            wrap=tk.WORD, # wrap at word boundaries
            yscrollcommand=scrollbar.set,
            font=('Courier', 10),
            width=15,
            state=tk.DISABLED # Disable editing by default
        )
        self.history_text.grid(row=0, column=0, sticky="nsew")
        
        # Link scrollbar to text widget
        scrollbar.config(command=self.history_text.yview)

        # Set up a protocol to handle closing the window
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _update_turn_color(self):
        """Internal helper to change the background color based on the current turn."""
        if self.current_turn == "WHITE":
            color = 'light gray'
            fg_color = 'black'
        else: # BLACK
            color = 'dark gray'
            fg_color = 'white'
            
        self.turn_label.config(background=color, foreground=fg_color)
        # self.turn_label.master.config(background=color) # This works if master is tk.Frame
        self.config(background=color) 

    def _update_display_and_toggle_turn(self, player, move_msg):
        """Called by ROS 2 callbacks to update all GUI elements safely."""
        move = move_msg.data
        
        # 1. Log the move
        self._log_move(player, move)
        
        # 2. Toggle Current Turn
        if player == "WHITE":
            self.last_white_move = move
            self.current_turn = "BLACK"
        else: # BLACK
            self.last_black_move = move
            self.current_turn = "WHITE"
            
        turn_string = "Twój ruch" if self.current_turn==self.human_pieces else "Ruch robota"
        self.turn_var.set(turn_string)
        self._update_turn_color()
        
        self.node.get_logger().info(f'GUI updated. New turn: {self.current_turn}. Last move: {move}')

    def _log_move(self, player, move):
        """Appends the move to the scrollable history log."""
        
        # Temporarily enable the Text widget for modification
        self.history_text.config(state=tk.NORMAL)
        
        if player == "WHITE":
            # Start a new move line: 1. e2e4
            log_entry = f"{self.move_number}. {move} "
            self.history_text.insert(tk.END, log_entry)
        else: # BLACK
            # Append black's move to the existing line: 1. e2e4 c7c5\n
            log_entry = f"{move}\n"
            self.history_text.insert(tk.END, log_entry)
            self.move_number += 1 # Increment move number only after Black moves

        # Scroll to the end to show the latest move
        self.history_text.see(tk.END)
        
        # Disable the Text widget again
        self.history_text.config(state=tk.DISABLED)

    def update_move_detection_info(self, msg):
        # 1. Determine Status, Color, and Detail Text
        if msg.move == "calibrated":
            status_text = "PRZETWARZANIE"
            status_color = "black"
            detail_text = "Brak ruchu"
        elif msg.legal:
            status_text = "WYKRYTO"
            status_color = "green"
            detail_text = f"Ruch: {msg.move}"
        elif len(msg.changed_squares) == 0:
            status_text = "PRZETWARZANIE"
            status_color = "black"
            detail_text = "Brak ruchu"
        else:
            status_text = "NIELEGALNY RUCH"
            status_color = "red"
            detail_text = f"Zmiany na polach: {msg.changed_squares}"
        
        # 2. Update the Indicator Label (Color & Text)
        self.detection_status_var.set(status_text)
        self.detection_label.config(foreground=status_color) # Change color here!

        # 3. Update the Detail Label
        self.detection_detail_var.set(detail_text)

        self.node.get_logger().info(f"Robot Status Updated:")

    def update_robot_status(self, status_msg):
        # 1. Determine Status, Color, and Detail Text
        if status_msg.success:
            status_text = "SUCCESS"
            status_color = "green"
            detail_text = f"Wykonany ruch: {status_msg.move}"
        else:
            status_text = "ERROR"
            status_color = "red"
            detail_text = f"Błąd przy wykonywaniu ruchu: {status_msg.move}\nPowód: {status_msg.error_message}"
        
        # 2. Update the Indicator Label (Color & Text)
        self.robot_status_indicator_var.set(status_text)
        self.robot_indicator_label.config(foreground=status_color) # Change color here!

        # 3. Update the Detail Label
        self.robot_status_detail_var.set(detail_text)

        self.node.get_logger().info(f"Robot Status Updated:")

    def update_white_move(self, msg):
        """Safely updates GUI after a /white_move received."""
        self._update_display_and_toggle_turn("WHITE", msg)

    def update_black_move(self, msg):
        """Safely updates GUI after a /black_move received."""
        self._update_display_and_toggle_turn("BLACK", msg)

    def update_game_result(self, msg):
        result = msg.data
        
        if result == "black_won":
            res_string = "Przegrana"
        elif result == "white_won":
            res_string = "Wygrana"
        elif result == "draw":
            res_string = "Remis"
        else:
            return
        
        self.turn_var.set(res_string)
        self.turn_label.config(background='light gray', foreground='black')
        # self.turn_label.master.config(background=color) # This works if master is tk.Frame
        self.config(background='light gray') 

    def publish_state_command(self, command_string):
        """Publishes the command string to the set_state topic."""
        
        if self.node is None or self.node.set_state_publisher_ is None:
            print("ERROR: ROS 2 node or publisher not initialized!")
            return

        msg = String()
        msg.data = command_string
        
        self.node.set_state_publisher_.publish(msg)
        self.node.get_logger().info(f"Published State Command: {command_string}")

    def on_closing(self):
        """Clean up and exit when the window is closed."""
        self.node.get_logger().info("GUI closed. Shutting down ROS 2 node.")
        self.node.destroy_node()
        rclpy.shutdown()
        self.destroy()

# --- 2. ROS 2 Node Class ---
class MoveListenerNode(Node):
    """ROS 2 Node to subscribe to /white_move and /black_move topics."""
    
    def __init__(self, gui):
        super().__init__('move_listener_node')
        self.gui = gui # Reference to the Tkinter GUI
        self.set_state_publisher_ = None

        # Subscription for White Moves
        self.subscription_white = self.create_subscription(
            String,
            '/white_move',
            self.white_move_callback,
            10
        )
        self.get_logger().info('Subscribed to /white_move')
        
        # Subscription for Black Moves
        self.subscription_black = self.create_subscription(
            String,
            '/black_move',
            self.black_move_callback,
            10
        )
        self.get_logger().info('Subscribed to /black_move')

        self.subscribtion_robot_status = self.create_subscription(
            RobotMoveStatus,
            '/robot_move_status',
            self.robot_status_callback,
            10
        )
        self.get_logger().info('Subscribed to /robot_move_status')

        self.subscribtion_robot_status = self.create_subscription(
            MoveDetectionStatus,
            '/move_detection_status',
            self.move_detection_status_callback,
            10
        )
        self.get_logger().info('Subscribed to /robot_move_status')


        self.subscription_black = self.create_subscription(
            String,
            '/game_result',
            self.game_result_callback,
            10
        )

    def initialize_publishers(self):
        """Creates the state publisher once the GUI is ready."""
        if self.set_state_publisher_ is None:
            self.set_state_publisher_ = self.create_publisher(
                String,
                '/set_state',
                10
            )
            self.get_logger().info('Initialized publisher for /set_state')
        
    def white_move_callback(self, msg):
        """Callback for /white_move topic."""
        self.get_logger().info(f'Received White Move: "{msg.data}"')
        # Use Tkinter's after() to safely update the GUI from the ROS 2 thread
        self.gui.after(0, self.gui.update_white_move, msg)

    def black_move_callback(self, msg):
        """Callback for /black_move topic."""
        self.get_logger().info(f'Received Black Move: "{msg.data}"')
        # Use Tkinter's after() to safely update the GUI from the ROS 2 thread
        self.gui.after(0, self.gui.update_black_move, msg)

    def robot_status_callback(self, msg):
        """Callback for /black_move topic."""
        self.get_logger().info(f'Received Robot Status: "{msg.move}"')
        # Use Tkinter's after() to safely update the GUI from the ROS 2 thread
        self.gui.after(0, self.gui.update_robot_status, msg)

    def move_detection_status_callback(self, msg):
        """Callback for /black_move topic."""
        self.get_logger().info(f'Received Move Detection: "{msg.move}"')
        # Use Tkinter's after() to safely update the GUI from the ROS 2 thread
        self.gui.after(0, self.gui.update_move_detection_info, msg)

    def game_result_callback(self, msg):
        """Callback for /black_move topic."""
        # Use Tkinter's after() to safely update the GUI from the ROS 2 thread
        self.gui.after(0, self.gui.update_game_result, msg)

# --- 3. Main Execution Functions ---

def ros_spin_thread(node):
    """Function to be run in a separate thread to spin the ROS 2 node."""
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"ROS 2 Spin Exception: {e}")
    finally:
        # If the spin loop exits (e.g., node destruction), clean up the GUI
        if node.gui.winfo_exists():
             node.gui.after(0, node.gui.on_closing)

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    # 1. Initialize Tkinter GUI
    gui = MoveDisplayGUI(None) # Node reference will be set after initialization

    # 2. Initialize ROS 2 Node
    move_listener_node = MoveListenerNode(gui)
    gui.node = move_listener_node # Set the node reference in the GUI
    move_listener_node.initialize_publishers()
    # 3. Run ROS 2 Spinning in a separate Thread
    # Tkinter must run in the main thread, so the ROS 2 node is moved to a background thread.
    ros_thread = threading.Thread(target=ros_spin_thread, args=(move_listener_node,), daemon=True)
    ros_thread.start()

    # 4. Run Tkinter Main Loop in the main thread
    try:
        gui.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if rclpy.ok():
            move_listener_node.destroy_node()
            rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()