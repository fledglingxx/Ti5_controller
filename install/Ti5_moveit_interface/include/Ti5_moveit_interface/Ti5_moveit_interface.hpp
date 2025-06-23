#pragma once

#include <rclcpp/rclcpp.hpp>   
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"



namespace Ti5_moveit_interface
{
    class MoveItInterface
    {
        public:
            MoveItInterface(const rclcpp::Node::SharedPtr &node,
                const std::string &planning_group_left,const std::string &planning_group_right);
            bool L_move_j(const std::vector<double> &joint_positions);
            bool L_move_p(const std::vector<double> &pose);
            bool R_move_j(const std::vector<double> &joint_positions);
            bool R_move_p(const geometry_msgs::msg::Pose &pose);

        private:
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> L_move_group_;
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> R_move_group_;

    };
}     // namespace Ti5_moveit_interface
