
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ur3_tcp/srv/gripper_cmd.hpp"
#include "ur3_tcp/srv/make_move.hpp"
#include "ur3_tcp/srv/move_joints.hpp"
#include "ur3_tcp/srv/move_named_pose.hpp"
#include "ur3_tcp/srv/move_waypoint.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

// using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;
using std::placeholders::_2;
namespace mtc = moveit::task_constructor;

enum MoveType {
  REGULAR,
  CAPTURE,
  CASTLE,
  ENPASSANT,
  PROMOTION,
  CAPTURE_PROMOTION
};

class ArmMoverTask : public rclcpp::Node {
 public:
  ArmMoverTask(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface() {
    return node_->get_node_base_interface();
  }

 private:
  bool moveWaypoit(const geometry_msgs::msg::Pose& waypoint);

  bool moveNamedPose(const std::string& pose);

  void makeMoveCallback(
      const std::shared_ptr<ur3_tcp::srv::MakeMove::Request> request,
      std::shared_ptr<ur3_tcp::srv::MakeMove::Response> response);

  geometry_msgs::msg::Pose squareToPose(const std::string& squareStr,
                                        bool placeDown);
  bool makeCapture(const std::string& squareStr, char piece);
  bool makeRegularMove(const std::string& fromSqr, const std::string& toSqr,
                       char piece);
  bool makePromotion(const std::string& toSqr, char piece);
  bool sendGripperCmd(const std::string& command);

  mtc::Task createWaypointTask(const geometry_msgs::msg::Pose& pose);
  mtc::Task createNamedPoseTask(const std::string& name);

  bool is_home_position_;
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  int promotionCount_ = 0;
  const std::string extraQueenSquares_[2] = {"^6", "^5"};

  rclcpp::CallbackGroup::SharedPtr gripper_cb_group_;
  rclcpp::Client<ur3_tcp::srv::GripperCmd>::SharedPtr gripper_cli;
  rclcpp::Service<ur3_tcp::srv::MakeMove>::SharedPtr makeMoveService_;
};

ArmMoverTask::ArmMoverTask(const rclcpp::NodeOptions& options)
    : Node("arm_mover_task", options) {
  // Create internal node for MTC with parameters
  node_ = std::make_shared<rclcpp::Node>("mtc_node", options);

  is_home_position_ = true;
  makeMoveService_ = create_service<ur3_tcp::srv::MakeMove>(
      "make_move", std::bind(&ArmMoverTask::makeMoveCallback, this, _1, _2));

  gripper_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  gripper_cli = create_client<ur3_tcp::srv::GripperCmd>(
      "gripper_cmd", rmw_qos_profile_services_default, gripper_cb_group_);
  RCLCPP_INFO(get_logger(), "ArmMoverTask service ready.");
}

void ArmMoverTask::makeMoveCallback(
    const std::shared_ptr<ur3_tcp::srv::MakeMove::Request> request,
    std::shared_ptr<ur3_tcp::srv::MakeMove::Response> response) {
  const MoveType moveType = static_cast<MoveType>(request->move_type);
  switch (moveType) {
    case MoveType::REGULAR: {
      bool success = true;
      success &= makeRegularMove(request->from_sqr, request->to_sqr,
                                 request->moved_piece);
      success &= moveNamedPose("idle");
      response->success = success;
      return;
    }
    case MoveType::CAPTURE: {
      bool success = true;
      success &= makeCapture(request->to_sqr, request->captured_piece);
      success &= makeRegularMove(request->from_sqr, request->to_sqr,
                                 request->moved_piece);
      success &= moveNamedPose("idle");
      response->success = success;
      return;
    }
    case MoveType::ENPASSANT: {
      const std::string capturedPieceSquare =
          std::string(1, request->to_sqr[0]) + request->from_sqr[1];
      bool success = true;
      success &= makeCapture(capturedPieceSquare, 'p');
      success &= makeRegularMove(request->from_sqr, request->to_sqr,
                                 request->moved_piece);
      success &= moveNamedPose("idle");
      response->success = success;
      break;
    }
    case MoveType::CASTLE: {
      std::string rookFromSquare;
      std::string rookToSquare;
      if (request->to_sqr == "g1") {
        rookFromSquare = "h1";
        rookToSquare = "f1";
      }
      if (request->to_sqr == "c1") {
        rookFromSquare = "a1";
        rookToSquare = "d1";
      }
      if (request->to_sqr == "g8") {
        rookFromSquare = "h8";
        rookToSquare = "f8";
      }
      if (request->to_sqr == "c8") {
        rookFromSquare = "a8";
        rookToSquare = "d8";
      }
      bool success = true;
      success &= makeRegularMove(request->from_sqr, request->to_sqr, 'k');
      success &= makeRegularMove(rookFromSquare, rookToSquare, 'r');
      success &= moveNamedPose("idle");
      response->success = success;
      break;
    }
    case MoveType::PROMOTION: {
      bool success = true;
      success &= makeCapture(request->from_sqr, request->moved_piece);
      success &= makePromotion(request->to_sqr, 'q');
      success &= moveNamedPose("idle");
      response->success = success;
      return;
    }
    case MoveType::CAPTURE_PROMOTION: {
      bool success = true;
      success &= makeCapture(request->to_sqr, request->captured_piece);
      success &= makeCapture(request->from_sqr, request->moved_piece);
      success &= makePromotion(request->to_sqr, 'q');
      success &= moveNamedPose("idle");
      response->success = success;
      return;
    }
  }
}

bool ArmMoverTask::makeCapture(const std::string& squareStr, char piece) {
  const geometry_msgs::msg::Pose capturedPiecePose =
      squareToPose(squareStr, false);
  bool success = true;

  success &= moveWaypoit(capturedPiecePose);
  success &= sendGripperCmd(std::string("close_") + piece);
  success &= moveNamedPose("out");
  success &= sendGripperCmd("open");
  return success;
}

bool ArmMoverTask::makeRegularMove(const std::string& fromSqr,
                                   const std::string& toSqr, char piece) {
  const geometry_msgs::msg::Pose fromPose = squareToPose(fromSqr, false);
  const geometry_msgs::msg::Pose toPose = squareToPose(toSqr, true);
  bool success = true;

  success &= moveWaypoit(fromPose);
  success &= sendGripperCmd(std::string("close_") + piece);
  success &= moveWaypoit(toPose);
  success &= sendGripperCmd("open");
  return success;
}

bool ArmMoverTask::makePromotion(const std::string& toSqr, char piece = 'q') {
  geometry_msgs::msg::Pose fromPose =
      squareToPose(extraQueenSquares_[promotionCount_], false);
  promotionCount_++;
  const float boardHeight = 0.024;
  fromPose.position.z -= boardHeight;
  const geometry_msgs::msg::Pose toPose = squareToPose(toSqr, true);
  bool success = true;

  success &= moveWaypoit(fromPose);
  success &= sendGripperCmd(std::string("close_") + piece);
  success &= moveWaypoit(toPose);
  success &= sendGripperCmd("open");
  return success;
}

bool ArmMoverTask::sendGripperCmd(const std::string& command) {
    auto request = std::make_shared<ur3_tcp::srv::GripperCmd::Request>();
    request->action = command;

    auto future = gripper_cli->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Gripper response timeout");
        return false;
    }

    auto resp = future.get();
    return resp->success;
}

geometry_msgs::msg::Pose ArmMoverTask::squareToPose(
    const std::string& squareStr, bool placeDown) {
  const float squareDimesions[2] = {0.037, 0.037};
  const float chessboardMidpoint[2] = {0.0, 0.3};

  const float boardHeight = 0.024;
  const float gripperToTool0Distance = 0.165;
  const float PieceGrippingDistance = 0.025;
  const float verticalMoveOffset = placeDown ? 0.105 : 0.1;

  const int row = squareStr[1] - '1';
  const int col = squareStr[0] - 'a';

  const float a1X = chessboardMidpoint[0] + (squareDimesions[0] * 3.5);
  const float a1Y = chessboardMidpoint[1] + (squareDimesions[1] * 3.5);

  geometry_msgs::msg::Pose pose;

  pose.position.x = a1X - col * squareDimesions[0];
  pose.position.y = a1Y - row * squareDimesions[1];
  pose.position.z = boardHeight + gripperToTool0Distance +
                    PieceGrippingDistance + verticalMoveOffset;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;

  return pose;
}

bool ArmMoverTask::moveWaypoit(const geometry_msgs::msg::Pose& waypoint) {
  task_ = createWaypointTask(waypoint);

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(get_logger(), e);
    return false;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Task planning failed");
    std::ofstream log_file("/home/igorsiata/mtc_failure.txt");
    if (log_file.is_open()) {
      log_file << "=== MTC Task Planning Failure ===" << std::endl;
      log_file << "Timestamp: " << std::time(nullptr) << std::endl;
      task_.explainFailure(log_file);
      log_file.close();
      RCLCPP_INFO(
          get_logger(),
          "Failure explanation written to /home/igorsiata/mtc_failure.txt");
      return false;
    } else {
      RCLCPP_ERROR(get_logger(), "Could not open log file");
      return false;
    }
  }

  auto& solutions = task_.solutions();
  auto best_it = std::min_element(
      solutions.begin(), solutions.end(),
      [](const auto& a, const auto& b) { return a->cost() < b->cost(); });

  auto best_solution = *best_it;

  task_.introspection().publishSolution(*best_solution);

  auto result = task_.execute(*best_solution);
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(get_logger(), "Task execution failed");
    return false;
  }
  is_home_position_ = false;
  RCLCPP_INFO(get_logger(), "End waypoint callback");
  return true;
}

bool ArmMoverTask::moveNamedPose(const std::string& pose) {
  task_ = createNamedPoseTask(pose);

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(get_logger(), e);
    return false;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Task planning failed");
    std::ofstream log_file("/home/igorsiata/mtc_failure.txt");
    if (log_file.is_open()) {
      log_file << "=== MTC Task Planning Failure ===" << std::endl;
      log_file << "Timestamp: " << std::time(nullptr) << std::endl;
      task_.explainFailure(log_file);
      log_file.close();
      RCLCPP_INFO(
          get_logger(),
          "Failure explanation written to /home/igorsiata/mtc_failure.txt");
      return false;
    } else {
      RCLCPP_ERROR(get_logger(), "Could not open log file");
      return false;
    }
  }

  auto& solutions = task_.solutions();
  auto best_it = std::min_element(
      solutions.begin(), solutions.end(),
      [](const auto& a, const auto& b) { return a->cost() < b->cost(); });

  auto best_solution = *best_it;

  task_.introspection().publishSolution(*best_solution);

  auto result = task_.execute(*best_solution);
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(get_logger(), "Task execution failed");
    return false;
  }
  return true;
}

mtc::Task ArmMoverTask::createNamedPoseTask(const std::string& name) {
  mtc::Task task;
  task.stages()->setName("go_home");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur_manipulator";
  const auto& hand_frame = "tool0";

  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr =
      nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto sampling_planner =
      std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner =
      std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_state_current =
      std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  if (!is_home_position_) {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat",
                                                             cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.099, 0.105);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 0.1;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }
  {
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("move_to_pose", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGroup(arm_group_name);
    stage->setGoal(name);
    task.add(std::move(stage));
  }
  if (name == "home" || name == "idle" || name == "out")
    is_home_position_ = true;
  return task;
}

mtc::Task ArmMoverTask::createWaypointTask(
    const geometry_msgs::msg::Pose& pose) {
  mtc::Task task;
  task.stages()->setName("pick_place");
  robot_model_loader::RobotModelLoader rml(node_);
  moveit::core::RobotModelConstPtr robot_model = rml.getModel();
  if (robot_model)
    task.setRobotModel(robot_model);
  else
    task.loadRobotModel(node_);

  const auto& arm_group_name = "ur_manipulator";
  const auto& hand_frame = "tool0";

  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in
// this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr =
      nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto sampling_planner =
      std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner =
      std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.05);

  auto stage_state_current =
      std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  if (!is_home_position_) {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat",
                                                             cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.099, 0.101);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat");

    // Set retreat direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.z = 0.1;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  {  // Go to target pose
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{
                             {arm_group_name, sampling_planner}});
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "world";
    target_pose_msg.pose = pose;

    auto move_to =
        std::make_unique<mtc::stages::GeneratePose>("move to target");
    move_to->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
    move_to->properties().set("marker_ns", "target_marker");
    move_to->setPose(target_pose_msg);
    move_to->setMonitoredStage(current_state_ptr);

    // Compute IK
    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK",
                                                            std::move(move_to));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(0.001);
    wrapper->setIKFrame(hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                            {"target_pose"});
    task.add(std::move(wrapper));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("aproach",
                                                             cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.099, 0.101);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "aproach");

    // Set retreat direction
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.z = -0.1;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<ArmMoverTask>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);
  executor.add_node(node->getNodeBaseInterface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
