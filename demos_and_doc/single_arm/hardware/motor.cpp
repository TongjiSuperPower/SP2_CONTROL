// Copyright 2020 ros2_control Development Team
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

#include "single_arm/motor.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <linux/can.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace single_arm
{
  constexpr double PI = 3.14159265358979;

  void Motor::read_thread_callback(const can_frame &rx_frame)
  {
    // std::cout << "CAN ID: " << std::hex << int(rx_frame.can_id) << ", Data: ";
    // for (int j = 0; j < rx_frame.can_dlc; ++j)
    // std::cout << std::hex << std::setfill('0') << std::setw(2) << int(rx_frame.data[j]) << " ";
    // std::cout << std::endl;
    std::lock_guard<std::mutex> lg(mutex_);
    this->read_buffer_ = rx_frame;
    return;
  }

  hardware_interface::CallbackReturn Motor::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    socket_can_.configure("can0", std::bind(&Motor::read_thread_callback, this, std::placeholders::_1));
    if (!socket_can_.open())
      // if (!socket_can_.open("can0", std::bind(&Motor::read_thread_callback, this, std::placeholders::_1)))
      return hardware_interface::CallbackReturn::SUCCESS;

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> Motor::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_state_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> Motor::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &effort_command_));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn Motor::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Motor::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Motor::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std::lock_guard<std::mutex> lg(mutex_);
    uint16_t ecd = (read_buffer_.data[0] << 8u) | read_buffer_.data[1];
    // 强转不会越界
    position_state_ = static_cast<double>(ecd) / 8192 * 2 * PI;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Motor::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    position_state_ = effort_command_;
    return hardware_interface::return_type::OK;
  }

} // namespace single_arm

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(single_arm::Motor, hardware_interface::SystemInterface)
