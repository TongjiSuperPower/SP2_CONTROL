#ifndef SP2_CONTROL_CHASSIS_BASE_HPP
#define SP2_CONTROL_CHASSIS_BASE_HPP

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <iostream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_box.h"
#include "control_toolbox/pid.hpp"

namespace chassis_controllers
{
    /**
     * @brief 底盘控制器基类
     */
    class ChassisBase : public controller_interface::ControllerInterface
    {
        using Twist = geometry_msgs::msg::TwistStamped;

    public:
        ChassisBase();

        virtual controller_interface::InterfaceConfiguration command_interface_configuration() const = 0;
        virtual controller_interface::InterfaceConfiguration state_interface_configuration() const = 0;

        virtual controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        virtual controller_interface::CallbackReturn on_init() = 0;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) = 0;
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) = 0;

    protected:
        bool is_subscriber_active_ = false;
        bool is_use_stamped_vel_ = false;

        double wheel_base_;
        double wheel_track_;
        double wheel_radius_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr nh_{};
        realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_unstamped_subscriber_ = nullptr;
        rclcpp::Subscription<Twist>::SharedPtr velocity_command_stamped_subscriber_ = nullptr;
        std::unordered_map<std::string, bool> configured_pid_gains_{};

        virtual controller_interface::CallbackReturn move_joints(
            geometry_msgs::msg::TwistStamped &command, const rclcpp::Duration &period) = 0;
        virtual void update_pids() = 0;
    };

} // namespace chassis_controllers

#endif