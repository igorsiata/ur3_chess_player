
#include <thread>
#include <memory>
#include <fstream>

#include "ur3_tcp/srv/move_waypoint.hpp"
#include "ur3_tcp/srv/move_named_pose.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
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

class ArmMoverTask : public rclcpp::Node
{
public:
    ArmMoverTask(const rclcpp::NodeOptions &options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }
    void doTask(const geometry_msgs::msg::Pose &pose);

private:
    void moveWaypoitCallback(
        const std::shared_ptr<ur3_tcp::srv::MoveWaypoint::Request> request,
        std::shared_ptr<ur3_tcp::srv::MoveWaypoint::Response> response);

    void moveNamedPoseCallback(
        const std::shared_ptr<ur3_tcp::srv::MoveNamedPose::Request> request,
        std::shared_ptr<ur3_tcp::srv::MoveNamedPose::Response> response);

    mtc::Task createWaypointTask(const geometry_msgs::msg::Pose &pose);
    mtc::Task createNamedPoseTask(const std::string &name);

    bool is_home_position_;
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    // MoveGroupInterface move_group_;

    rclcpp::Service<ur3_tcp::srv::MoveWaypoint>::SharedPtr move_waypoint_service_;
    rclcpp::Service<ur3_tcp::srv::MoveNamedPose>::SharedPtr move_named_pose_service_;
};

ArmMoverTask::ArmMoverTask(const rclcpp::NodeOptions &options)
    : Node("arm_mover_task", options)
{
    // Create internal node for MTC with parameters
    node_ = std::make_shared<rclcpp::Node>("mtc_node", options);

    is_home_position_ = true;

    move_waypoint_service_ = create_service<ur3_tcp::srv::MoveWaypoint>(
        "move_waypoint",
        std::bind(&ArmMoverTask::moveWaypoitCallback, this, _1, _2));
    move_named_pose_service_ = create_service<ur3_tcp::srv::MoveNamedPose>(
        "move_named_pose",
        std::bind(&ArmMoverTask::moveNamedPoseCallback, this, _1, _2));
    RCLCPP_INFO(get_logger(), "ArmMoverTask service ready.");
}

void ArmMoverTask::moveWaypoitCallback(
    const std::shared_ptr<ur3_tcp::srv::MoveWaypoint::Request> request,
    std::shared_ptr<ur3_tcp::srv::MoveWaypoint::Response> response)
{
    task_ = createWaypointTask(request->waypoint);

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR_STREAM(get_logger(), e);
        response->success = false;
        return;
    }

    if (!task_.plan(2))
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Task planning failed");
        std::ofstream log_file("/home/igorsiata/mtc_failure.txt");
        if (log_file.is_open())
        {
            log_file << "=== MTC Task Planning Failure ===" << std::endl;
            log_file << "Timestamp: " << std::time(nullptr) << std::endl;
            task_.explainFailure(log_file);
            log_file.close();
            RCLCPP_INFO(get_logger(), "Failure explanation written to /home/igorsiata/mtc_failure.txt");
            response->success = false;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not open log file");
            response->success = false;
        }
        return;
    }

    auto &solutions = task_.solutions();
    auto best_it = std::min_element(
        solutions.begin(), solutions.end(),
        [](const auto &a, const auto &b)
        { return a->cost() < b->cost(); });

    auto best_solution = *best_it;

    task_.introspection().publishSolution(*best_solution);

    auto result = task_.execute(*best_solution);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Task execution failed");
        response->success = false;

        return;
    }
    is_home_position_ = false;
    response->success = true;
    return;
}

void ArmMoverTask::moveNamedPoseCallback(
    const std::shared_ptr<ur3_tcp::srv::MoveNamedPose::Request> request,
    std::shared_ptr<ur3_tcp::srv::MoveNamedPose::Response> response)
{
    task_ = createNamedPoseTask(request->name);

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR_STREAM(get_logger(), e);
        response->success = false;
        return;
    }

    if (!task_.plan(5))
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Task planning failed");
        std::ofstream log_file("/home/igorsiata/mtc_failure.txt");
        if (log_file.is_open())
        {
            log_file << "=== MTC Task Planning Failure ===" << std::endl;
            log_file << "Timestamp: " << std::time(nullptr) << std::endl;
            task_.explainFailure(log_file);
            log_file.close();
            RCLCPP_INFO(get_logger(), "Failure explanation written to /home/igorsiata/mtc_failure.txt");
            response->success = false;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not open log file");
            response->success = false;
        }
        return;
    }

    auto &solutions = task_.solutions();
    auto best_it = std::min_element(
        solutions.begin(), solutions.end(),
        [](const auto &a, const auto &b)
        { return a->cost() < b->cost(); });

    auto best_solution = *best_it;

    task_.introspection().publishSolution(*best_solution);

    auto result = task_.execute(*best_solution);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Task execution failed");
        response->success = false;

        return;
    }
    response->success = true;
    return;
}

mtc::Task ArmMoverTask::createNamedPoseTask(const std::string &name)
{
    mtc::Task task;
    task.stages()->setName("go_home");
    task.loadRobotModel(node_);

    const auto &arm_group_name = "ur_manipulator";
    const auto &hand_frame = "gripper_tcp_sphere";

    task.setProperty("group", arm_group_name);
    task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    if(!is_home_position_){
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.015, 0.025);
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "retreat");

        // Set retreat direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.z = 0.02;
        stage->setDirection(vec);
        task.add(std::move(stage));
    }

    { // Go to target pose
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
        stage_move_to_place->setTimeout(5.0);
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_place));
    }
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>(name, sampling_planner);
        stage->setGroup(arm_group_name);
        stage->setGoal(name);
        task.add(std::move(stage));
    }
    return task;
}

mtc::Task ArmMoverTask::createWaypointTask(const geometry_msgs::msg::Pose &pose)
{
    mtc::Task task;
    task.stages()->setName("pick_place");
    robot_model_loader::RobotModelLoader rml(node_);
    moveit::core::RobotModelConstPtr robot_model = rml.getModel();
    if (robot_model)
        task.setRobotModel(robot_model);
    else
        task.loadRobotModel(node_);

    const auto &arm_group_name = "ur_manipulator";
    // const auto &hand_group_name = "ur_gripper";
    const auto &hand_frame = "tool0";

    // Set task properties
    task.setProperty("group", arm_group_name);
    // If you have a real gripper group, set it here and provide named poses like "open"/"close".
    // task.setProperty("hand", hand_group_name);
    // task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.05);

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    if (!is_home_position_){
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.015, 0.025);
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "retreat");

        // Set retreat direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "base_link";
        vec.vector.z = 0.02;
        stage->setDirection(vec);
        task.add(std::move(stage));
    }

    { // Go to target pose
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    {
        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "world"; // choose your reference frame
        target_pose_msg.pose = pose;

        auto move_to = std::make_unique<mtc::stages::GeneratePose>("move to target");
        move_to->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
        move_to->properties().set("marker_ns", "target_marker");
        move_to->setPose(target_pose_msg);
        move_to->setMonitoredStage(current_state_ptr);

        // Compute IK
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(move_to));
        wrapper->setMaxIKSolutions(5);
        wrapper->setMinSolutionDistance(0.001);
        wrapper->setIKFrame(hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        task.add(std::move(wrapper));
    }
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("aproach", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.045, 0.055);
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "aproach");

        // Set retreat direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "base_link";
        vec.vector.z = -0.05;
        stage->setDirection(vec);
        task.add(std::move(stage));
    }
    return task;
}

int main(int argc, char *argv[])
{
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
