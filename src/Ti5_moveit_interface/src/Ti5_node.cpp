#include "Ti5_moveit_interface/Ti5_moveit_interface.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Ti5_moveit_interface_node");
  
  Ti5_moveit_interface::MoveItInterface Ti5(node, "L_group", "R_group");

  std::vector<double> left_joints = {0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0};
  Ti5.L_move_j(left_joints);

  std::vector<double> right_joints = {0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0};
  Ti5.R_move_j(right_joints);

  while(rclcpp::ok())
  { 
    rclcpp::spin(node);
  }

  // rclcpp::shutdown();
  return 0;
}