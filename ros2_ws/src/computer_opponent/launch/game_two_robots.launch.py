from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    white_player = Node(
        package="computer_opponent",
        executable="stockfish_node",
        name="stockfish_node_white",
        parameters=[{"is_white": True, "skill_level": 5}],
    )

    # Second node
    black_player = Node(
        package="computer_opponent",
        executable="stockfish_node",
        name="stockfish_node_black",
        parameters=[{"is_white": False, "skill_level": 5}],
    )

    move_proxy = Node(
        package="computer_opponent",
        executable="move_proxy",
        name="move_proxy_node",
    )

    # Only start node2 after node1 starts
    delayed_white_player = RegisterEventHandler(
        OnProcessStart(
            target_action=move_proxy,
            on_start=[
                TimerAction(
                    period=0.5,
                    actions=[white_player],  # wait 0.5s to ensure node1 initialized
                )
            ],
        )
    )

    delayed_black_player = RegisterEventHandler(
        OnProcessStart(
            target_action=move_proxy,
            on_start=[
                TimerAction(
                    period=0.5,
                    actions=[black_player],  # wait 0.5s to ensure node1 initialized
                )
            ],
        )
    )

    return LaunchDescription([move_proxy, delayed_black_player, delayed_white_player])
