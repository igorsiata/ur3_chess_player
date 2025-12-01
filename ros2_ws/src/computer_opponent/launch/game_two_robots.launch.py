from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    start_position_arg = DeclareLaunchArgument(
        "start_position",
        default_value="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
        description="Game starting position",
    )
    stockfish_skill_lvl_arg = DeclareLaunchArgument(
        "stockfish_skill_lvl", default_value="5", description="Level of stockfish"
    )

    white_player = Node(
        package="computer_opponent",
        executable="stockfish_node",
        name="stockfish_node_white",
        parameters=[
            {
                "is_white": True,
                "skill_level": LaunchConfiguration("stockfish_skill_lvl"),
                "start_position": LaunchConfiguration("start_position"),
            }
        ],
    )

    # Second node
    black_player = Node(
        package="computer_opponent",
        executable="stockfish_node",
        name="stockfish_node_black",
        parameters=[
            {
                "is_white": False,
                "skill_level": LaunchConfiguration("stockfish_skill_lvl"),
                "start_position": LaunchConfiguration("start_position"),
            }
        ],
    )

    return LaunchDescription(
        [
            stockfish_skill_lvl_arg,
            start_position_arg,
            black_player,
            white_player,
        ]
    )
