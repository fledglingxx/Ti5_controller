// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "Ti5_hardware_interface/hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace Ti5_hardware_interface
{
hardware_interface::CallbackReturn hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // TODO(anyone): read parameters and initialize the hardware
  info_ = info;

  joint_data_.resize(info_.joints.size());
  motor_pos_feedback_.resize(info_.joints.size());
  motor_vel_feedback_.resize(info_.joints.size());
  motor_eff_feedback_.resize(info_.joints.size());

  node_ = rclcpp::Node::make_shared("robot_system_hw");
  motor_pos_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("data_analysis/motor_pos",1);
  motor_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("data_analysis/motor_vel",1);
  motor_torque_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("data_analysis/motor_torque",1);
  RCLCPP_INFO(node_->get_logger(), "Ti5 hardware interface initialized");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
  
  int cnt=3;
  while(cnt--)
  {
    if(init_can())
    {
      RCLCPP_INFO(node_->get_logger(), "CAN initialized successfully");
      break;
    }
  }
  if(cnt==0)  
  {
    RCLCPP_ERROR(node_->get_logger(), "CAN initialization failed");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {

    RCLCPP_INFO(node_->get_logger(), "Exporting state interface for joint %s", info_.joints[i].name.c_str());
    state_interfaces.emplace_back(info_.joints[i].name, "position", &joint_data_[i].pos_);
    state_interfaces.emplace_back(info_.joints[i].name, "velocity", &joint_data_[i].vel_);
    state_interfaces.emplace_back(info_.joints[i].name, "effort", &joint_data_[i].eff_);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(info_.joints[i].name, "position", &joint_data_[i].pos_cmd_);
    command_interfaces.emplace_back(info_.joints[i].name, "velocity", &joint_data_[i].vel_cmd_);
    command_interfaces.emplace_back(info_.joints[i].name, "effort", &joint_data_[i].eff_cmd_);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type hardware::read(const rclcpp::Time & , const rclcpp::Duration & )
{
  for(size_t i=0; i<info_.joints.size(); i++)
  {
    joint_data_[i].pos_ = rv_motor_msg[i].position;
    joint_data_[i].vel_ = rv_motor_msg[i].velocity;
    joint_data_[i].eff_ = rv_motor_msg[i].torque;

    motor_pos_feedback_(i) = joint_data_[i].pos_;
    motor_vel_feedback_(i) = joint_data_[i].vel_;
    motor_eff_feedback_(i) = joint_data_[i].eff_;
  }
  
  motor_pos_pub_->publish(createFloat64MultiArrayFromVector(motor_pos_feedback_));
  motor_vel_pub_->publish(createFloat64MultiArrayFromVector(motor_vel_feedback_));
  motor_torque_pub_->publish(createFloat64MultiArrayFromVector(motor_eff_feedback_));

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type hardware::write(const rclcpp::Time & , const rclcpp::Duration & )
{
  ////////////sendCanCommand();

  return hardware_interface::return_type::OK;
}

}  // namespace Ti5_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  Ti5_hardware_interface::hardware, hardware_interface::SystemInterface)
