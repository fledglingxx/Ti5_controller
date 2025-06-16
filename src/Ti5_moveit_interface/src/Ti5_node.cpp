#include "Ti5_moveit_interface/Ti5_moveit_interface.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Ti5_moveit_interface_node");
  
  Ti5_moveit_interface::MoveItInterface Ti5(node, "left_arm", "right_arm");

  std::vector<double> left_joints = {0.0, -0.5, 1.0, 0.0, 0.7, 0.0, 0.0};
  Ti5.move_left_arm(left_joints);

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.2;
  pose.orientation.w = 0.0;
  Ti5.R_move_p(pose);


  rclcpp::shutdown();
  return 0;
}