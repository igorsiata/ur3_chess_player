from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    start_position_arg = DeclareLaunchArgument(
        "start_position",
        default_value="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
        description="Game starting position",
    )
    stockfish_skill_lvl_arg = DeclareLaunchArgument(
        "stockfish_skill_lvl", default_value="5", description="Level of stockfish"
    )
    human_is_white_arg = DeclareLaunchArgument(
        "human_is_white",
        default_value="true",
        description="Is human playing as white pieces",
    )

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[
            {
                "video_device": "/dev/video0",
                "pixel_format": "mjpg",
                "image_size": [1280, 720],
                "framerate": 5,
                # "contrast": 10,
                # "sharpness": 5,
                # "saturation": 50,
                # "white_balance_automatic": False,
                # "white_balance_temperature": 4500,
            }
        ],
    )

    human_player = Node(
        package="computer_opponent",
        executable="human_move_detection",
        name="human_move_detection_node",
        parameters=[
            {
                "is_white": True,
                "skill_level": LaunchConfiguration("stockfish_skill_lvl"),
                "start_position": LaunchConfiguration("start_position"),
            }
        ],
    )

    # Second node
    robot_player = Node(
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

    move_proxy = Node(
        package="computer_opponent",
        executable="move_proxy",
        name="move_proxy_node",
    )

    # Only start node2 after node1 starts
    delayed_robot_player = RegisterEventHandler(
        OnProcessStart(
            target_action=move_proxy,
            on_start=[
                TimerAction(
                    period=0.5,
                    actions=[robot_player],  # wait 0.5s to ensure node1 initialized
                )
            ],
        )
    )

    delayed_human_player = RegisterEventHandler(
        OnProcessStart(
            target_action=move_proxy,
            on_start=[
                TimerAction(
                    period=0.5,
                    actions=[human_player],  # wait 0.5s to ensure node1 initialized
                )
            ],
        )
    )

    return LaunchDescription(
        [
            human_is_white_arg,
            stockfish_skill_lvl_arg,
            start_position_arg,
            move_proxy,
            camera_node,
            delayed_human_player,
            delayed_robot_player,
        ]
    )
