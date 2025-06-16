#pragma once

#include <rclcpp/erclcpp.hpp>   
#include <moveit/movegroup_interface/movve_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace Ti5_moveit_interface
{
    class MoveItInterface
    {
        public:
            MoveItInterface(const rclcpp::Node::SharedPtr &node, const std::string &planning_group);
            bool L_move_j(const std::vector<double> &joint_positions);
            bool L_move_p(const geometry_msgs::msg::Pose &pose);
            bool R_move_j(const std::vector<double> &joint_positions);
            bool R_move_p(const geometry_msgs::msg::Pose &pose);

        private:
            rclcpp::Node::SharedPtr node_;
            moveit::planning_interface::MoveGroupInterface::SharedPtr L_move_group_;
            moveit::planning_interface::MoveGroupInterface::SharedPtr R_move_group_;
    };
}     // namespace Ti5_moveit_interface
