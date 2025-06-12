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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "Ti5_arms_controller/Ti5_arms_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace Ti5_arms_controller
{
  Ti5ArmsController::Ti5ArmsController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn Ti5ArmsController::on_init()
  {
    auto node = get_node();

    node->declare_parameter<std::vector<std::string>>("joint_names");
    node->declare_parameter<std::vector<std::string>>("joint_command_interfaces");
    node->declare_parameter<std::vector<std::string>>("joint_state_interfaces");

    node->get_parameter("joint_names",joint_names_);
    node->get_parameter("joint_command_interfaces",joint_command_interfaces_);
    node->get_parameter("joint_state_interfaces",joint_state_interfaces_);
    
    RCLCPP_INFO(node->get_logger(), "Loaded %zu joints", joint_names_.size());
                                                                                  
    return controller_interface::CallbackReturn::SUCCESS;
  }



  controller_interface::CallbackReturn Ti5ArmsController::on_configure(const rclcpp_lifecycle::State &)
  {

    if(joint_names_.empty() || joint_command_interfaces_.empty() || joint_state_interfaces_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint names, command interfaces or state interfaces not set. Cannot start controller.")
      return controller_interface::CallbackReturn::ERROR;
    }

    command_interfaces_handles_.resize(joint_names_.size());
    for(const auto &joint_name : joint_names_)
    {
      for(const auto &interface_type : joint_command_interfaces_)
      {
        auto handle = get_interface(joint_name, interface_type, command_interfaces_);
        if(!handle)
        {
          RCLCPP_ERROR(get_node()->get_logger(), "command interface %s for joint %s not available",
                        interface_type.c_str(), joint_name.c_str());
          return controller_interface::CallbackReturn::ERROR;
        }
        command_interface_handles_.push_back(handle);
      }
    }
    
    state_interfaces_handles_.resize(joint_names_.size());
    for(const auto &joint_name : joint_names_)
    {
      for(const auto &interface_type : joint_state_interfaces_)
      {
        auto handle = get_interface(joint_name, interface_type, state_interfaces_);
        if(!handle)
        {
          RCLCPP_ERROR(get_node()->get_logger(), "state interface %s for joint %s not available",
                        interface_type.c_str(), joint_name.c_str());
          return controller_interface::CallbackReturn::ERROR;
        }
        state_interface_handles_.push_back(handle);
      }
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }




  controller_interface::InterfaceConfiguration Ti5ArmsController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reverse(joint_names_.size());
    for(const auto &joint : joint_names_)
    {
      for(const auto &interface : joint_command_interfaces_)
        command_interfaces_config.names.push_back(joint + "/" + interface);
    }

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration Ti5ArmsController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(joint_names_.size());
    for (const auto &joint : joint_names_)
    {
      for(const auto &interface : joint_state_interfaces_)
        state_interfaces_config.names.push_back(joint + "/" + interface);
    }

    return state_interfaces_config;
  }




  controller_interface::CallbackReturn Ti5ArmsController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
    // `on_activate` method in `JointTrajectoryController` for exemplary use of
    // `controller_interface::get_ordered_interfaces` helper function

    // Set default value in command
    reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn Ti5ArmsController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
    // instead of a loop
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type Ti5ArmsController::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    auto current_ref = input_ref_.readFromRT();

    // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
    // instead of a loop
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      if (!std::isnan((*current_ref)->displacements[i]))
      {
        if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
        {
          (*current_ref)->displacements[i] /= 2;
        }
        command_interfaces_[i].set_value((*current_ref)->displacements[i]);

        (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
      }
    }

    if (state_publisher_ && state_publisher_->trylock())
    {
      state_publisher_->msg_.header.stamp = time;
      state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
      state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

} // namespace Ti5_arms_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    Ti5_arms_controller::Ti5ArmsController, controller_interface::ControllerInterface)
