// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef TI5_ARMS_CONTROLLER__TI5_ARMS_CONTROLLER_HPP_
#define TI5_ARMS_CONTROLLER__TI5_ARMS_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "Ti5_arms_controller_parameters.hpp"
#include "Ti5_arms_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_interface/helpers.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace Ti5_arms_controller
{

  class Ti5ArmsController : public controller_interface::ControllerInterface
  {
  public:
    TI5_ARMS_CONTROLLER__VISIBILITY_PUBLIC
    Ti5ArmsController();

    TI5_ARMS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    TI5_ARMS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    TI5_ARMS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    TI5_ARMS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    TI5_ARMS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    TI5_ARMS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    TI5_ARMS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // TODO(anyone): replace the state and command message types
    using ControllerReferenceMsg = control_msgs::msg::JointJog;
    using ControllerModeSrvType = std_srvs::srv::SetBool;
    using ControllerStateMsg = control_msgs::msg::JointControllerState;

private:
    std::shared_ptr<Ti5_arms_controller::ParamListener> param_listener_;
    Ti5_arms_controller::Params params_;
  
    std::vector<std::string> joint_command_interfaces_;
    std::vector<std::string> joint_state_interfaces_;


    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interfaces_;
    std::vector<std::string> state_interfaces_;

    std::vector<hardware_interface::LoanedCommandInterface> command_interface_handles_;
    std::vector<hardware_interface::LoanedStateInterface> state_interface_handles_;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharePtr joint_trajectory_sub_;
    std::vector<double> target_positions_;

    std::mutex target_mutex_;

  };

} // namespace Ti5_arms_controller

#endif // TI5_ARMS_CONTROLLER__TI5_ARMS_CONTROLLER_HPP_
