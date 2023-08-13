#ifndef SP2_CONTROL_CHASSIS_BASE_HPP
#define SP2_CONTROL_CHASSIS_BASE_HPP

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_box.h"

#include "roller_wheel_controller_parameter.hpp"

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
            const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

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
        realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_unstamped_subscriber_ = nullptr;
        rclcpp::Subscription<Twist>::SharedPtr velocity_command_stamped_subscriber_ = nullptr;
    };

} // namespace chassis_controllers

namespace chassis_controllers
{
    class RollerWheel : public chassis_controllers::ChassisBase
    {
    public:
        RollerWheel();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::CallbackReturn on_init() override;
        /**
         * @brief 完成参数读取
         */
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;
        /**
         * @brief 获取已经储存在state_interface_和command_interface_中的资源
         */
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
        struct WheelHandle
        {
            std::string joint_name;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> vel_state;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> eff_cmd;
        };

        std::shared_ptr<roller_wheel::ParamListener> param_listener_;
        roller_wheel::Params params_;
        std::vector<WheelHandle> registered_wheel_handles_;

        controller_interface::CallbackReturn configure_joints(
            const std::vector<std::string> &wheel_name, std::vector<WheelHandle> &registered_handles);
        controller_interface::CallbackReturn move_joints(geometry_msgs::msg::TwistStamped &command);

    private:
        std::vector<std::string> wheel_names_{};
    };

} // namespace chassis_controllers
#endif