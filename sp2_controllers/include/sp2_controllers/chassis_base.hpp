#ifndef SP2_CONTROL_CHASSIS_BASE_HPP
#define SP2_CONTROL_CHASSIS_BASE_HPP

#include "controller_interface/controller_interface.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_box.h"

namespace ChassisBase
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

        virtual controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
        bool is_subscriber_active_ = false;
        bool is_use_stamped_vel_ = false;
        realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_unstamped_subscriber_ = nullptr;
        rclcpp::Subscription<Twist>::SharedPtr velocity_command_stamped_subscriber_ = nullptr;
    };

} // namespace ChassisBase

namespace RollerWheel
{
    class RollerWheel : public ChassisBase::ChassisBase
    {
    public:
        RollerWheel();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;
    };

} // namespace RollerWheel
#endif