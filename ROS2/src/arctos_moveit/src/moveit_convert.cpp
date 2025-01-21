#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "arctos_msgs/msg/joint_angles.hpp"
#include "arctos_msgs/msg/joint_state.hpp"
#include "arctos_msgs/msg/robot_pose.hpp"

class MoveitConvert : public rclcpp::Node
{
public:
    MoveitConvert() : Node("moveit_convert")
    {
        joint_pub_ = this->create_publisher<arctos_msgs::msg::JointAngles>("joint_angles", 10);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_pose", 10,
            std::bind(&MoveitConvert::poseCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "MoveitConvert node initialized");
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Convert pose to joint angles using inverse kinematics
        auto joint_msg = std::make_unique<arctos_msgs::msg::JointAngles>();
        // TODO: Implement inverse kinematics
        joint_pub_->publish(std::move(joint_msg));
    }

    rclcpp::Publisher<arctos_msgs::msg::JointAngles>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitConvert>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
