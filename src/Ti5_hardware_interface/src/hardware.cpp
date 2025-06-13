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
  hardware_interface::CallbackReturn hardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    num_joints_ = info.joints.size();

    joint_names_.resize(num_joints_);
    pos_cmd_.resize(num_joints_, 0.0);
    vel_cmd_.resize(num_joints_, 0.0);
    eff_cmd_.resize(num_joints_, 0.0);

    pos_state_.resize(num_joints_, 0.0);
    vel_state_.resize(num_joints_, 0.0);
    eff_state_.resize(num_joints_, 0.0);

    RCLCPP_INFO(node_->get_logger(), "Hardware interface initialized successfully");
    return CallbackReturn::SUCCESS;
  }


  hardware_interface::CallbackReturn hardware::on_configure(const rclcpp_lifecycle::State &)
  {
    int cnt = 3;
    while (cnt--)
    {
      if (init_can())
      {
        RCLCPP_INFO(node_->get_logger(), "CAN initialized successfully");
        break;
      }
    }
    if (cnt == 0)
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
      state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_state_[i]);
      state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);
      state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &eff_state_[i]);
    }
    return state_interfaces;
  }


  std::vector<hardware_interface::CommandInterface> hardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < num_joints_; ++i)
    {
      command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
      command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
      command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &eff_cmd_[i]);
    }

    return command_interfaces;
  }


  hardware_interface::CallbackReturn hardware::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(node_->get_logger(), "Activating hardware interface");

    for(size_t i = 0; i < num_joints_; i++)
    {
      pos_sate_[i] = 0.0;
      vel_state_[i] = 0.0;
      eff_state_[i] = 0.0;

      pos_cmd_[i] = pos_state_[i];
      vel_cmd_[i] = 0.0;
      eff_cmd_[i] = 0.0;
    }

    return CallbackReturn::SUCCESS;
  }


  hardware_interface::CallbackReturn hardware::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(node_->get_logger(), "Deactivating hardware interface");

    return CallbackReturn::SUCCESS;
  }


  hardware_interface::return_type hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for(size_t i = 0; i < num_joints_; i++)
    {
      pos_state_[i] = 1.0;
      vel_state_[i] = 0.0;  
      eff_state_[i] = 0.0;
    }

    return hardware_interface::return_type::OK;
  }



  hardware_interface::return_type hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // send can commands

    for(size_t i=0; i<num_joints_; i++)
    {
      RCLCPP_DEBUG(node_->get_logger("Ti5_hardware_interface"), "Writing position command for joint %s: %f", 
              joint_names_[i].c_str(), pos_cmd_[i]);

    return hardware_interface::return_type::OK;
  }

} // namespace Ti5_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    Ti5_hardware_interface::hardware, hardware_interface::SystemInterface)
