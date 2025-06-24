#include "Ti5_moveit_interface/Ti5_moveit_interface.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Ti5_moveit_interface_node");
  
  Ti5_moveit_interface::MoveItInterface Ti5(node, "L_group", "R_group");

  std::vector<double> left_joints = {0.5,0.1,0.3,0.5,0.3,0.2,0.3};
  std::vector<double> left_pos = {0.0689,0.6335,0.1913,0.9706,0.3198,0.3198};
  Ti5.L_move_j(left_joints);
  Ti5.L_move_p(left_pos);


  std::vector<double> right_joints = {0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0};
  std::vector<double> right_pos = {0.1196,-0.626,-0.0539,0.8728,-1.156,-0.0639};
  Ti5.R_move_j(right_joints);
  Ti5.R_move_p(right_pos);

  while(rclcpp::ok())
  { 
    rclcpp::spin(node);
  }

  // rclcpp::shutdown();
  return 0;
}