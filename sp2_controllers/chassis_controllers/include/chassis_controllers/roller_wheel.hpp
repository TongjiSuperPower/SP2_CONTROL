#ifndef SP2_CONTROL_ROLLER_WHEEL_HPP
#define SP2_CONTROL_ROLLER_WHEEL_HPP

#include "chassis_controllers/chassis_base.hpp"
#include "roller_wheel_controller_parameter.hpp"

namespace chassis_controllers
{
    class RollerWheel : public chassis_controllers::ChassisBase
    {
    public:
        RollerWheel();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

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
        controller_interface::CallbackReturn move_joints(
            geometry_msgs::msg::TwistStamped &command, const rclcpp::Duration &period) override;

    private:
        size_t dof_ = 4;
        std::vector<std::string> wheel_names_;
        std::vector<std::shared_ptr<control_toolbox::Pid>> pids_;
        std::vector<double> wheel_vel_command_;

        rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
        void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
        void update_pids();
    };

} // namespace chassis_controllers
#endif