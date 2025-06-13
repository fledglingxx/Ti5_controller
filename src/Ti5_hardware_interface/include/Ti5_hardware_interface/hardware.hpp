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

#ifndef TI5_HARDWARE_INTERFACE__HARDWARE_HPP_
#define TI5_HARDWARE_INTERFACE__HARDWARE_HPP_

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "Ti5_hardware_interface/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <rclcpp/rclcpp.hpp>

#include "can_hw.h"

using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;

struct Ti5MotorData
{
  double pos_, vel_, eff_;
  double pos_cmd_, vel_cmd_, eff_cmd_;
};

namespace Ti5_hardware_interface
{


  inline std_msgs::msg::Float64MultiArray createFloat64MultiArrayFromVector(const vector_t &data)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(data.data(), data.data() + data.size());
    return msg;
  }



  class hardware : public hardware_interface::SystemInterface
  {
  public:
    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    size_t num_joints_;
    std::vector<std::string> joint_names_;
    std::vector<double> pos_cmd_, vel_cmd_, eff_cmd_;
    std::vector<double> pos_state_, vel_state_, eff_state_;

  };

} // namespace Ti5_hardware_interface

#endif // TI5_HARDWARE_INTERFACE__HARDWARE_HPP_
