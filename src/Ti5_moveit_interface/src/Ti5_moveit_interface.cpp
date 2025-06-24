#include "Ti5_moveit_interface/Ti5_moveit_interface.hpp"

namespace Ti5_moveit_interface
{
    MoveItInterface::MoveItInterface(const rclcpp::Node::SharedPtr &node,
                                     const std::string &left_group,const std::string &right_group) : node_(node)
    {

        L_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, left_group);
        R_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, right_group);

        L_end_effector_link_ = L_move_group_->getEndEffectorLink();
        L_move_group_->setPlanningTime(5.0);
        L_move_group_->setMaxVelocityScalingFactor(0.5);
        L_move_group_->setMaxAccelerationScalingFactor(0.5);

        R_end_effector_link_ = R_move_group_->getEndEffectorLink();
        R_move_group_->setPlanningTime(5.0);
        R_move_group_->setMaxVelocityScalingFactor(0.5);
        R_move_group_->setMaxAccelerationScalingFactor(0.5);

    }
    

    bool MoveItInterface::L_move_j(const std::vector<double> &joint_positions)
    {

        std::cout<<"hhhhhhhhhhh!!!!!!!   L_move_j"<<std::endl;

        L_move_group_->setStartStateToCurrentState();
        L_move_group_->setJointValueTarget(joint_positions);
        L_move_group_->move();
        return true;
    }

    bool MoveItInterface::L_move_p(const std::vector<double> &pose)
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = pose[0];
        target_pose.position.y = pose[1];
        target_pose.position.z = pose[2];

        tf2::Quaternion q;
        q.setRPY(pose[3], pose[4], pose[5]);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();

        L_move_group_->setStartStateToCurrentState();
        L_move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = L_move_group_->plan(plan);
        RCLCPP_INFO(rclcpp::get_logger("Ti5_moveit_interface"), "Plan %s", success? "successful" : "failed");

        if(success)
        {
            L_move_group_->execute(plan);
            return true;
        }
        return false;

    }


    bool MoveItInterface::R_move_j(const std::vector<double> &joint_positions)
    {

        std::cout<<"hhhhhhhhhhh!!!!!!!   R_move_j"<<std::endl;

        R_move_group_->setStartStateToCurrentState();
        R_move_group_->setJointValueTarget(joint_positions);
        R_move_group_->move();
        return true;
    }

    bool MoveItInterface::R_move_p(const std::vector<double> &pose)
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = pose[0];
        target_pose.position.y = pose[1];
        target_pose.position.z = pose[2];

        tf2::Quaternion q;
        q.setRPY(pose[3], pose[4], pose[5]);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();

        R_move_group_->setStartStateToCurrentState();
        R_move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = R_move_group_->plan(plan);
        RCLCPP_INFO(rclcpp::get_logger("Ti5_moveit_interface"), "Plan %s", success ? "successful" : "failed");

        if(success)
        {
            R_move_group_->execute(plan);
            return true;
        }
        return false;

    }


    void MoveItInterface::get_L_pos()
    {
        geometry_msgs::msg::PoseStamped current_pose = L_move_group_->getCurrentPose(L_end_effector_link_);
        if (current_pose.header.stamp.sec == 0 && current_pose.header.stamp.nanosec == 0) 
        {
            RCLCPP_WARN(rclcpp::get_logger("Ti5_moveit_interface"), "L pose not available, timestamp invalid.");
            return;
        }

        std::vector<double> rpy = L_move_group_->getCurrentRPY(L_end_effector_link_);
        RCLCPP_INFO(rclcpp::get_logger("Ti5_moveit_interface"),"L pose: x=%.3f, y=%.3f, z=%.3f | roll=%.3f, pitch=%.3f, yaw=%.3f",
                current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,rpy[0], rpy[1], rpy[2]);
    }


    void MoveItInterface::get_R_pos()
    {
        geometry_msgs::msg::PoseStamped current_pose = R_move_group_->getCurrentPose(R_end_effector_link_);
        if (current_pose.header.stamp.sec == 0 && current_pose.header.stamp.nanosec == 0) 
        {
            RCLCPP_WARN(rclcpp::get_logger("Ti5_moveit_interface"), "R pose not available, timestamp invalid.");
            return;
        }

        std::vector<double> rpy = R_move_group_->getCurrentRPY(R_end_effector_link_);
        RCLCPP_INFO(rclcpp::get_logger("Ti5_moveit_interface"),"R pose: x=%.3f, y=%.3f, z=%.3f | roll=%.3f, pitch=%.3f, yaw=%.3f",
                current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,rpy[0], rpy[1], rpy[2]);
    }   
}