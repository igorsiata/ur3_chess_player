import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory("ur_tcp_moveit_config")

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_tcp_moveit_config")
        .robot_description(file_path=os.path.join(pkg_share, "srdf", "ur.urdf.xacro"),
                            mappings={
            "name": "ur",
        })
        .robot_description_semantic(
            file_path=os.path.join(pkg_share, "srdf", "ur.srdf.xacro"),
                            mappings={
            "name": "ur",
        })
        .trajectory_execution(file_path=os.path.join(pkg_share, "config", "controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Capabilities for move_group
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # ROS2 Control node using FakeSystem
    ros2_controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Move group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz node
    rviz_config_file = os.path.join(pkg_share, "config", "mtc.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # Static transform (world → tool0)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "tool0"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
    ]:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "controller_manager",
                    "spawner",
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        )

    return LaunchDescription(
        [
            ros2_control_node,
            robot_state_publisher,
            static_tf,
            run_move_group_node,
            rviz_node,
        ]
        + load_controllers
    )
