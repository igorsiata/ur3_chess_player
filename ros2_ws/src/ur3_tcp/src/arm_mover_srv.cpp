#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "ur3_tcp/srv/move_waypoint.hpp"
#include "ur3_tcp/srv/move_named_pose.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;
using std::placeholders::_2;

class ArmMover : public rclcpp::Node
{
public:
    ArmMover()
        : Node("arm_mover"),
          move_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {}), "ur_manipulator")
    {
        move_group_.setPoseReferenceFrame("base_link");
        move_group_.allowReplanning(true);
        move_group_.setNumPlanningAttempts(10);
        move_group_.setGoalTolerance(0.001);
        move_group_.setGoalOrientationTolerance(0.001);
        move_group_.setGoalPositionTolerance(0.001);
        move_group_.setMaxVelocityScalingFactor(0.3);
        move_group_.setMaxAccelerationScalingFactor(0.3);
        move_group_.setPlanningTime(10.0);

        // Create service
        move_waypoint_service_ = create_service<ur3_tcp::srv::MoveWaypoint>(
            "move_waypoint",
            std::bind(&ArmMover::move_waypoints_callback, this, _1, _2));

        move_named_pose_service_ = create_service<ur3_tcp::srv::MoveNamedPose>(
            "move_named_pose",
            std::bind(&ArmMover::move_named_pose_callback, this, _1, _2));

        RCLCPP_INFO(get_logger(), "ArmMover service ready.");
    }

private:
    void move_waypoints_callback(
        const std::shared_ptr<ur3_tcp::srv::MoveWaypoint::Request> request,
        std::shared_ptr<ur3_tcp::srv::MoveWaypoint::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Received target pose for planning.");
        geometry_msgs::msg::Pose target_pose = request->waypoint;
        geometry_msgs::msg::Pose target_pose_above = target_pose;
        target_pose_above.position.z += 0.1;

        if(!move_pose(target_pose_above)){
            response->success = false;
            response->message = "Moving over object failed";
            return;
        }

            // Define waypoints: only move down in Z
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;       // Resolution of trajectory in meters
        const double jump_threshold = 0.0;  // Disable jump threshold (optional)
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(get_logger(), "Path computed (%.2f%% achieved)", fraction * 100.0);

        if (fraction > 0.99)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group_.execute(plan);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Cartesian path incomplete.");
        }

        response->success = true;
        response->message = "Success";
        return;

    }

    void move_named_pose_callback(const std::shared_ptr<ur3_tcp::srv::MoveNamedPose::Request> request,
        std::shared_ptr<ur3_tcp::srv::MoveNamedPose::Response> response){
        if(!move_named_target(request->name)){
            response->success = false;
            return;
        }
        response->success = true;
        return;
    }

    bool move_pose(const geometry_msgs::msg::Pose &target_pose)
    {
        const char *gripper_joint = "tool0";
        // move_group_.setStartStateToCurrentState();
        move_group_.setPoseTarget(target_pose, gripper_joint);

        MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                return true;
        }
        return false;
    }

    bool move_named_target(const std::string &target)
    {
        move_group_.setNamedTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                return true;
        }
        return false;
    }

    MoveGroupInterface move_group_;
    rclcpp::Service<ur3_tcp::srv::MoveWaypoint>::SharedPtr move_waypoint_service_;
    rclcpp::Service<ur3_tcp::srv::MoveNamedPose>::SharedPtr move_named_pose_service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMover>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
