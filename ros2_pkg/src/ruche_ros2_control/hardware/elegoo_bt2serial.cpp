// Copyright 2021 ros2_control Development Team
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

// The includes will reflect the path after building the package
// . is install/pkg_name/shared/.
#include "include/elegoo_bt2serial.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_bt_ser_interface
{
hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.device_id = info_.hardware_parameters["device_id"];
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  wheel_l_.name = cfg_.left_wheel_name;
  wheel_r_.name = cfg_.right_wheel_name;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // The Elegoo bot has no states and one command interface on each joint
    // It has 4 wheels but only 2 can be controlled
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElegooBt2SerialHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElegooBt2SerialHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Predisposition for state feedback
// std::vector<hardware_interface::StateInterface> ElegooBt2SerialHardware::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   state_interfaces.emplace_back(hardware_interface::StateInterface(
//     wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
//   state_interfaces.emplace_back(hardware_interface::StateInterface(
//     wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

//   state_interfaces.emplace_back(hardware_interface::StateInterface(
//     wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
//   state_interfaces.emplace_back(hardware_interface::StateInterface(
//     wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

//   return state_interfaces;
// }

std::vector<hardware_interface::CommandInterface> ElegooBt2SerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElegooBt2SerialHardware"), "Configuring ...please wait...");
  // TODO: Open bluetooth connection
  
  RCLCPP_INFO(rclcpp::get_logger("ElegooBt2SerialHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElegooBt2SerialHardware"), "Cleaning up ...please wait...");
  // TODO: close connection
  RCLCPP_INFO(rclcpp::get_logger("ElegooBt2SerialHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElegooBt2SerialHardware"), "Activating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("ElegooBt2SerialHardware"), "Successfully Activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElegooBt2SerialHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("ElegooBt2SerialHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// hardware_interface::return_type ElegooBt2SerialHardware::read(
//   const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
// {
  
//   return hardware_interface::return_type::OK;
// }

hardware_interface::return_type ros2_control_bt_ser_interface ::ElegooBt2SerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_bt_ser_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_bt_ser_interface::ElegooBt2SerialHardware, hardware_interface::SystemInterface)