cmake_minimum_required(VERSION 3.8)
project(ur3_tcp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  tf2_eigen
)
# -----------------------------------------------------------------------------
# Interface generation
# -----------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveWaypoint.srv"
  "srv/MoveNamedPose.srv"
  "srv/MoveJoints.srv"
  "srv/MakeMove.srv"
  "srv/GripperCmd.srv"
  "action/MovePiece.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------------------------------------------------------
# Executables
# -----------------------------------------------------------------------------
add_executable(arm_mover_srv src/arm_mover_srv.cpp)
ament_target_dependencies(arm_mover_srv
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_core
  tf2_geometry_msgs
)

# add_executable(arm_mover src/arm_mover.cpp)
# ament_target_dependencies(arm_mover
#   rclcpp
#   moveit_ros_planning_interface
#   geometry_msgs
#   moveit_core
#   tf2_geometry_msgs
# )

add_executable(arm_mover_task src/arm_mover_task.cpp)
ament_target_dependencies(arm_mover_task
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_core
  tf2_geometry_msgs
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Get the typesupport target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link the executable to it
target_link_libraries(arm_mover_srv ${cpp_typesupport_target})
# target_link_libraries(arm_mover ${cpp_typesupport_target})
target_link_libraries(arm_mover_task ${cpp_typesupport_target})
# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
install(
  DIRECTORY config launch meshes rviz urdf srv action
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  # arm_mover
  arm_mover_srv
  arm_mover_task
  DESTINATION lib/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
