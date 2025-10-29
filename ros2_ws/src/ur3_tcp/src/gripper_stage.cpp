#include <moveit/task_constructor/stage.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

namespace mtc = moveit::task_constructor;

class GripperCommandStage : public mtc::PropagatingEitherWay
{
public:
    GripperCommandStage(const std::string &name
                        // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                        // const std::string &cmd
                    )
         : mtc::PropagatingEitherWay(name) //client_(std::move(client)), command_(cmd)
    {
        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    }
    void computeForward(const mtc::InterfaceState &from) override
    {
        auto result = apply(from, false);
        sendForward(from, std::move(result.first), std::move(result.second));
    };
    void computeBackward(const mtc::InterfaceState &to) override
    {
        auto result = apply(to, true);
        sendBackward(std::move(result.first), to, std::move(result.second));
    };
    void closeGripper(){
        RCLCPP_INFO(rclcpp::get_logger("Gripper command state"), "CLOSING GRIPPER");
    };

    void openGripper(){
        RCLCPP_INFO(rclcpp::get_logger("Gripper command state"), "OPENING GRIPPER");
    };

private:
    std::pair<mtc::InterfaceState, mtc::SubTrajectory>
    apply(const mtc::InterfaceState &from, bool invert)
    {
        // auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        // req->command = 
        // auto future = client_->async_send_request(req);

        mtc::InterfaceState state(from);
        mtc::SubTrajectory trajectory;

        // if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("GripperCommandStage"),
        //                  "Gripper command timeout");
        //     trajectory.markAsFailure("Service call timed out");
        //     return {state, trajectory};
        // }

        // auto resp = future.get();
        // auto resp = future.get();
        // if (!resp->success)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("GripperCommandStage"),
        //                  "Gripper command failed: %s", resp->message.c_str());
        //     trajectory.markAsFailure(resp->message);
        //     return {state, trajectory};
        // }

        // Mark this SubTrajectory as a successful non-motion command
        RCLCPP_INFO(rclcpp::get_logger("Gripper command state"), "CLOSING GRIPPER");
        trajectory.setComment("Gripper command executed");
        return {state, trajectory};
    };
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string command_;
};