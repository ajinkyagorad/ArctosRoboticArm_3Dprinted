#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

class MoveGroupInterface : public rclcpp::Node
{
public:
    MoveGroupInterface() : Node("move_group_interface")
    {
        // Create the MoveIt MoveGroup Interface
        using namespace std::chrono_literals;
        
        // Allow time for other nodes to start
        std::this_thread::sleep_for(2s);
        
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this), "arm");
            
        // Create a publisher for visualization
        robot_state_pub_ = this->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1);
        
        // Create a subscriber for target poses
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&MoveGroupInterface::poseCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "MoveGroup Interface Node initialized");
        
        // Set end effector link
        move_group_interface_->setEndEffectorLink("Link_6_1");
        
        // Get current pose
        geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getCurrentPose();
        
        // Create a publisher for current pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);
        
        // Publish current pose
        pose_pub_->publish(current_pose);
        
        // Create a visual tools object
        namespace rvt = rviz_visual_tools;
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            std::shared_ptr<rclcpp::Node>(this),
            "base_link",
            "moveit_visual_markers",
            move_group_interface_->getRobotModel()
        );
        
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();
        
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.0;
        visual_tools_->publishText(text_pose, "MoveGroupInterface Moveo Demo", rvt::WHITE, rvt::XLARGE);
        visual_tools_->trigger();
        
        // Create a target pose
        geometry_msgs::msg::Pose target_pose1;
        target_pose1.position.x = 0.120679;
        target_pose1.position.y = 0.072992;
        target_pose1.position.z = 0.569166;
        target_pose1.orientation.x = -0.386473;
        target_pose1.orientation.y = -0.418023;
        target_pose1.orientation.z = -0.760978;
        target_pose1.orientation.w =  0.311139;
        
        // Set target pose
        move_group_interface_->setPoseTarget(target_pose1);
        
        // Plan to target pose
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing movement");
            move_group_interface_->execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
        
        // Visualize target pose
        visual_tools_->publishAxisLabeled(target_pose1, "pose1");
        visual_tools_->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools_->trigger();
        visual_tools_->prompt("Execute trajectory");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped target_pose = *msg;
        move_group_interface_->setPoseTarget(target_pose.pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing movement");
            move_group_interface_->execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto move_group_interface = std::make_shared<MoveGroupInterface>();
    rclcpp::spin(move_group_interface);
    rclcpp::shutdown();
    return 0;
}
