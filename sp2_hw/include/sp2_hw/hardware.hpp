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
        double effort_command_;
        double position_state_;
        TYPE2ACTCOEFF_MAP type2act_coeff_;

        template <typename typeT>
        void setActCoeffMap(const std::string &type_name, const typeT &type, TYPE2ACTCOEFF_MAP &map);
    };
} // namespace SP2Control

#endif