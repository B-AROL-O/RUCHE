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
#include "ruche_ros2_control/elegoo_bt2serial.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <cstddef>
#include <iomanip>
#include <sstream>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sdbus-c++/sdbus-c++.h>
#include <iostream>
#include <format>
#include <string>
#include <fmt/core.h>

namespace ruche_ros2_control
{
  hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params)
  {
    if (
        hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read parameters from the xml
    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.device_id = info_.hardware_parameters["device_id"];
    cfg_.if_name = "org.bluez.GattCharacteristic1";
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    // reset values always when configuring hardware
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, 0.0);
    }

    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    // command and state should be equal when starting
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, get_state(name));
    }

    RCLCPP_INFO(get_logger(), "Connecting to the robot ...please wait...");
    const std::string deviceAddr = cfg_.device_id; // D-Bus format
    const std::string serviceCharName = "service0013/char0017";
    const std::string serviceName = "org.bluez";
    const std::string adapterPath = "/org/bluez/hci0";
    const std::string interfaceName = cfg_.if_name;
    const std::string devicePath = "/org/bluez/hci0/dev_" + deviceAddr;
    const std::string servicePath = devicePath + "/" + serviceCharName;

    try
    {

      auto connection = sdbus::createSystemBusConnection();
      auto rootProxy = sdbus::createProxy(*connection, serviceName, "/");
      auto adapterProxy = sdbus::createProxy(*connection, serviceName, adapterPath); // /hc01
      connection->enterEventLoopAsync();

      adapterProxy->callMethod("StartDiscovery")
          .onInterface("org.bluez.Adapter1");
      RCLCPP_INFO(get_logger(), "Starting bluetooth discovery, wait...");

      // Wait for the device to be discovered. Open loop
      sleep(3);

      // Stop discovery
      adapterProxy->callMethod("StopDiscovery").onInterface("org.bluez.Adapter1");

      // Connect to device
      auto devProxy = sdbus::createProxy(*connection, serviceName, devicePath);
      devProxy->callMethod("Connect")
          .onInterface("org.bluez.Device1");
      RCLCPP_INFO(get_logger(), "Connecting to the bluetooth device...");
      usleep(500000);

      // Wait for ServicesResolved
      bool resolved = false;
      while (!resolved)
      {
        auto prop = devProxy->getProperty("ServicesResolved").onInterface("org.bluez.Device1");
        resolved = prop.get<bool>();
        usleep(500000);
      }
      RCLCPP_INFO(get_logger(), "Resolve device services...");

      RCLCPP_INFO(get_logger(), "Open connection on service %s and path %s", serviceName.c_str(), servicePath.c_str());
      serviceProxy_ = sdbus::createProxy(*connection, serviceName, servicePath);
      // auto serviceProxy = sdbus::createProxy(*connection, serviceName, servicePath);

      // Prepare data to send
      std::string str = "Init done!";
      std::vector<uint8_t> value(str.begin(), str.end());

      RCLCPP_INFO(get_logger(), "Sending test data...");
      serviceProxy_->callMethod("WriteValue")
          .onInterface(interfaceName)
          .withArguments(value, std::map<std::string, sdbus::Variant>{});
    }
    catch (const sdbus::Error &e)
    {
      RCLCPP_FATAL(
          get_logger(), "D-Bus error: %s - %s", e.getName().c_str(), e.getMessage().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ElegooBt2SerialHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Disonnecting to the robot ...please wait...");
    const std::string deviceAddr = cfg_.device_id; // D-Bus format
    const std::string serviceName = "org.bluez";
    const std::string devicePath = "/org/bluez/hci0/dev_" + deviceAddr;

    try
    {

      auto connection = sdbus::createSystemBusConnection();

      // Connect to device
      auto devProxy = sdbus::createProxy(*connection, serviceName, devicePath);
      devProxy->callMethod("Disconnect")
          .onInterface("org.bluez.Device1");
    }
    catch (const sdbus::Error &e)
    {
      RCLCPP_FATAL(
          get_logger(), "D-Bus error: %s - %s", e.getName().c_str(), e.getMessage().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ElegooBt2SerialHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    // No feedback from robot. Otherwise the states are read here
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ruche_ros2_control ::ElegooBt2SerialHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std::ostringstream oss;
    double v;
    double v_l;
    double v_r;
    const std::string interfaceName = cfg_.if_name;
    // Read controls
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      v = get_command(name);
      json_cmd_[name] = v;

      // send left message
      if (name.rfind(cfg_.left_wheel_name, 0) == 0)
      {
        v_l = v;
      }
      else if (name.rfind(cfg_.right_wheel_name, 0) == 0)
      {
        v_r = v;
      }
    }
    oss << "v_l: " << std::fixed << std::setprecision(2) << v_l << "; v_r: " << std::fixed << std::setprecision(2) << v_r << std::endl;
    std::string str_send = oss.str();
    std::vector<uint8_t> value(str_send.begin(), str_send.end());
    // RCLCPP_INFO(get_logger(), "\n\nWriting %s", str_send.c_str());

    serviceProxy_->callMethod("WriteValue")
        .onInterface(interfaceName)
        .withArguments(value, std::map<std::string, sdbus::Variant>{});

    // TODO: remove this!
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "\r\nCommands received: %s", json_cmd_.dump().c_str());

    return hardware_interface::return_type::OK;
  }

} // namespace ruche_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ruche_ros2_control::ElegooBt2SerialHardware, hardware_interface::SystemInterface)
