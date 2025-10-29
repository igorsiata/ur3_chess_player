#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
using std::placeholders::_1;

class ArmMover : public rclcpp::Node
{
public:
    ArmMover()
        : Node("arm_mover"),
          move_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {}), "ur_manipulator")
    {
        RCLCPP_INFO(get_logger(), "Initializing ArmMover...");

        move_group_.setPoseReferenceFrame("base_link");
        move_group_.setPlannerId("RRTConnectkConfigDefault");
        move_group_.setNumPlanningAttempts(10);
        move_group_.setPlanningTime(5.0);
        move_group_.setGoalTolerance(0.001);
        move_group_.setGoalOrientationTolerance(0.001);
        move_group_.setGoalPositionTolerance(0.0001);
        move_group_.setMaxVelocityScalingFactor(0.3);
        move_group_.setMaxAccelerationScalingFactor(0.3);

        RCLCPP_INFO(get_logger(), "Pose reference frame set to: %s", move_group_.getPoseReferenceFrame().c_str());

        subscription_ = create_subscription<geometry_msgs::msg::Pose>(
            "robot_ee_pos", 10, std::bind(&ArmMover::topic_callback, this, _1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Pose &target_pose)
    {
        RCLCPP_INFO(get_logger(), "Received target pose for planning.");
        
        move_group_.setStartStateToCurrentState();
        auto joint_values = move_group_.getCurrentJointValues();
        move_group_.setJointValueTarget(joint_values);
        move_group_.setPlannerId("RRTConnectkConfigDefault");

        

        move_group_.setPoseTarget(target_pose, "tool0");

        RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_.getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "End effector link: %s", move_group_.getEndEffectorLink().c_str());

        MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(get_logger(), "Plan found, executing...");
            move_group_.move();
            RCLCPP_INFO(get_logger(), "Motion execution completed.");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Motion planning failed!");
        }
    }

    MoveGroupInterface move_group_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMover>();

    // Use MultiThreadedExecutor to avoid blocking callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
