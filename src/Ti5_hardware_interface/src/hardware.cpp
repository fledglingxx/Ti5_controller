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


#include <iostream>
#include <cstring>
#include <chrono>

#include "../include/Ti5_hardware_interface/hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace Ti5_hardware_interface
{
  hardware_interface::CallbackReturn hardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    num_joints_ = info.joints.size();

    hw_commands_.resize(num_joints_,0);
    hw_positions_.resize(num_joints_,0);
    hw_velocities.resize(num_joints_,0);
    joint_names_.clear();

    for(const auto &joint : info.joints)
      {
        joint_names_.push_back(joint.name);
        std::cout<<"Joint name: "<<joint.name<<std::endl;
      }
    
    RCLCPP_INFO(rclcpp::get_logger("Ti5_hardware_interface"), "Hardware interface initialized successfully");
    return CallbackReturn::SUCCESS;
  }


  hardware_interface::CallbackReturn hardware::on_configure(const rclcpp_lifecycle::State &)
  {
    can_motor_interface = std::make_shared<CANMotorInterface>();
    int cnt = 3;
    while (cnt--)
    {
      if (can_motor_interface->initCAN())
      {
        RCLCPP_INFO(rclcpp::get_logger("Ti5_hardware_interface"), "CAN initialized successfully");
        break;
      }
    }
    if (cnt == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Ti5_hardware_interface"), "CAN initialization failed");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }




  std::vector<hardware_interface::StateInterface> hardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (int i = 0; i < num_joints_; ++i)
    {
      state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities[i]);
    }

    return state_interfaces;
  }


  std::vector<hardware_interface::CommandInterface> hardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (int i = 0; i < num_joints_; ++i)
      command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);

    return command_interfaces;
  }


  hardware_interface::CallbackReturn hardware::on_activate(const rclcpp_lifecycle::State &)
  {

    for(int i = 0; i < num_joints_; i++)
    {
      std::cout<<"hhhhhhhhhhhhhhhhhhh!!!!!!!!!!! Activating joint: "<<i<<std::endl;
      can_motor_interface->set_vel(i, 36, 1000);
      can_motor_interface->set_vel(i, 37, -1000);
    }

    RCLCPP_INFO(rclcpp::get_logger("Ti5_hardware_interface"), "Activating hardware interface");
    return CallbackReturn::SUCCESS;
  }


  hardware_interface::CallbackReturn hardware::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("Ti5_hardware_interface"), "Deactivating hardware interface");

    return CallbackReturn::SUCCESS;
  }


  hardware_interface::return_type hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for(int i = 0; i < num_joints_; i++)
    {
      hw_positions_[i] = can_motor_interface->sendSimpleCanCommand(i,8);
      hw_velocities[i] = can_motor_interface->sendSimpleCanCommand(i,6);

    }

    return hardware_interface::return_type::OK;
  }



  hardware_interface::return_type hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
  {

    for(int i=0; i<num_joints_; i++)
    {

      can_motor_interface->sendCanCommand(i, 30, hw_commands_[i]);

      RCLCPP_DEBUG(rclcpp::get_logger("Ti5_hardware_interface"), "Writing position command for joint %s: %f", 
               joint_names_[i].c_str(), hw_commands_[i]);

    }
    return hardware_interface::return_type::OK;

  } // namespace Ti5_hardware_interface
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    Ti5_hardware_interface::hardware, hardware_interface::SystemInterface)