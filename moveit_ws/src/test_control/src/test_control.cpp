#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "test_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("test_control");

    // Next step goes here
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    // Set a target Pose
    std::vector<geometry_msgs::msg::Pose> target_poses;

    geometry_msgs::msg::Pose pose1;
    pose1.orientation.w = 1.0;
    pose1.position.x = 0.28;
    pose1.position.y = -0.2;
    pose1.position.z = 0.5;
    target_poses.push_back(pose1);

    geometry_msgs::msg::Pose pose2;
    pose2.orientation.w = 1.0;
    pose2.position.x = 0.0;
    pose2.position.y = 0.0;
    pose2.position.z = 0.0;
    target_poses.push_back(pose2);

    rclcpp::Rate rate(0.2);
    size_t index = 0;

    while (rclcpp::ok())
    {
        auto const &target_pose = target_poses[index];
        move_group_interface.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface.plan(plan));

        if (success)
        {
            move_group_interface.execute(plan);
            RCLCPP_INFO(logger, "Executed plan to pose index %zu", index);
        }
        else
        {
            RCLCPP_ERROR(logger, "Planning to pose index %zu failed!", index);
        }

        // Advance index cyclically
        if (index == target_poses.size() - 1)
            index = 0;

        rate.sleep();
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}