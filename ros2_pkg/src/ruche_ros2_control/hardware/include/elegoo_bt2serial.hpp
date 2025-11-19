

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

#ifndef ROS2_CONTROL_BLUETOOTH_SERIAL_INTERFACE_
#define ROS2_CONTROL_BLUETOOTH_SERIAL_INTERFACE_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

namespace ros2_control_bt_ser_interface
{
class ElegooBt2SerialHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  std::string device_id = "";
  int timeout_ms = 0;
};

struct Wheel
{
  std::string name = "";
  double cmd = 0.0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ElegooBt2SerialHardware)

  ELEGOO_BT2SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // ELEGOO_BT2SERIAL_PUBLIC
  // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ELEGOO_BT2SERIAL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ELEGOO_BT2SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ELEGOO_BT2SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  ELEGOO_BT2SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ELEGOO_BT2SERIAL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ELEGOO_BT2SERIAL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ELEGOO_BT2SERIAL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
};

}  // namespace ros2_control_bt_ser_interface

#endif  // ROS2_CONTROL_BLUETOOTH_SERIAL_INTERFACE_