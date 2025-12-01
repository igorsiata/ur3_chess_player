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

    # camera_node = Node(
    #     package="realsense2_camera",
    #     executable="realsense2_camera_node",
    #     name="d455_camera",
    #     output="screen",
    #     parameters=[
    #         {
    #             "enable.depth": False,
    #             "pointcloud.enable": False,
    #             "rgb_camera.color_profile": "1280x720x5",
    #         }
    #     ],
    # )
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[
            {
                "video_device": "/dev/video0",
                # "pixel_format": "mjpg",
                "image_size": [1920, 1080],
                "framerate": 30,
            }
        ],
    )

    human_player_white = Node(
        package="computer_opponent",
        executable="human_move_detection",
        name="human_white",
        parameters=[
            {
                "is_white": True,
                "start_position_fen": LaunchConfiguration("start_position"),
            }
        ],
    )

    human_player_black = Node(
        package="computer_opponent",
        executable="human_move_detection",
        name="human_black",
        parameters=[
            {
                "is_white": False,
                "start_position_fen": LaunchConfiguration("start_position"),
            }
        ],
    )

    delayed_white_player = TimerAction(
        period=0.5,
        actions=[human_player_white],  # wait 0.5s to ensure node1 initialized
    )

    delayed_black_player = TimerAction(
        period=0.5,
        actions=[human_player_black],  # wait 0.5s to ensure node1 initialized
    )


    return LaunchDescription(
        [
            stockfish_skill_lvl_arg,
            start_position_arg,
            camera_node,
            delayed_black_player,
            delayed_white_player,
        ]
    )
