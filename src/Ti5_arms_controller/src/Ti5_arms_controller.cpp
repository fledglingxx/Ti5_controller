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

    node->get_parameter("joint_names", joint_names_);
    node->get_parameter("joint_command_interfaces", joint_command_interfaces_);
    node->get_parameter("joint_state_interfaces", joint_state_interfaces_);

    RCLCPP_INFO(node->get_logger(), "Loaded %zu joints", joint_names_.size());

    return controller_interface::CallbackReturn::SUCCESS;
  }



  controller_interface::CallbackReturn Ti5ArmsController::on_configure(const rclcpp_lifecycle::State &)
  {
    if (joint_names_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint names not set. Cannot configure controller.");
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Controller configured successfully (simplified).");
    return controller_interface::CallbackReturn::SUCCESS;
  }



  controller_interface::InterfaceConfiguration Ti5ArmsController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(joint_names_.size() * joint_command_interfaces_.size());
    for (const auto &joint : joint_names_)
    {
      for (const auto &interface : joint_command_interfaces_)
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
      for (const auto &interface : joint_state_interfaces_)
        state_interfaces_config.names.push_back(joint + "/" + interface);
    }

    return state_interfaces_config;
  }

  controller_interface::CallbackReturn Ti5ArmsController::on_activate(const rclcpp_lifecycle::State &)
  {
    trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory", 10,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (msg->points.empty()) return;

        const auto& last_point = msg->points.back();
        target_positions_ = last_point.positions;

        RCLCPP_INFO(get_node()->get_logger(), "Received joint trajectory with %zu positions.", target_positions_.size());
      }

    )

    RCLCPP_INFO(get_node()->get_logger(), "activating Ti5 arms controller");
    return controller_interface::CallbackReturn::SUCCESS;
  }



  controller_interface::CallbackReturn Ti5ArmsController::on_deactivate(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_node()->get_logger(), "deactivating Ti5 arms controller");
    return controller_interface::CallbackReturn::SUCCESS;
  }






  controller_interface::return_type Ti5ArmsController::update(const rclcpp::Time &time, const rclcpp::Duration &)
  {
    std::lock_guard<std::mutex> lock(target_mutex_);

    if(target_positions_.size() != joint_names_.size())
    {
      RCLCPP_WARN(get_node()->get_logger(),"Target jolint size mismatch.");
      return controller_interface::return_type::ERROR;
    }

    for(size_t i = 0; i < joint_names_.size(); i++)
    {
      command_interfaces_[i].get().set_value(target_positions_[i]);
    }
    

    return controller_interface::return_type::OK;
  }

} // namespace Ti5_arms_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    Ti5_arms_controller::Ti5ArmsController, controller_interface::ControllerInterface)
