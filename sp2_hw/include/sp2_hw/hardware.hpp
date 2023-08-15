/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */
#ifndef SP2_CONTROL_HARDWARE_HPP
#define SP2_CONTROL_HARDWARE_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "sp2_hw/hardware_interface/CanBus.hpp"
// parse YAML
#include "yaml-cpp/yaml.h"
#include "sp2_hw/config.h"
#include <fstream>
// transmission
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

namespace SP2Control
{
    class SP2Hardware : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::mutex mutex_;

        TYPE2ACTCOEFF_MAP type2act_coeff_;
        // 未来可能有非can总线电机，因此名字并没有被定位can_bus2act_data_
        std::unordered_map<std::string, ID2ACTDATA_MAP> bus_name2act_data_{};
        std::vector<std::unique_ptr<CanBus>> can_buses_;
        // Transmission
        std::vector<JntData> jnt_data_;
        std::unordered_map<std::string, JntData *> jnt_name2jnt_data_ptr_{};
        std::vector<std::shared_ptr<transmission_interface::Transmission>> jnt2act_transmissions_;
        std::vector<std::shared_ptr<transmission_interface::Transmission>> act2jnt_transmissions_;
    };
} // namespace SP2Control

#endif