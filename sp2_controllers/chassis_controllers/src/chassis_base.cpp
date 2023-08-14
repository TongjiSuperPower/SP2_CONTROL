#include "chassis_controllers/chassis_base.hpp"

namespace chassis_controllers
{
    ChassisBase::ChassisBase() : controller_interface::ControllerInterface(){};

    controller_interface::CallbackReturn ChassisBase::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        const Twist empty_twist;
        received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));
        // 订阅Twist话题
        if (is_use_stamped_vel_)
        {
            velocity_command_stamped_subscriber_ = get_node()->create_subscription<Twist>(
                "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
                [this](const std::shared_ptr<Twist> msg) -> void
                {
                    if (!is_subscriber_active_)
                    {
                        RCLCPP_WARN(
                            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                        return;
                    }
                    if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                    {
                        RCLCPP_WARN_ONCE(
                            get_node()->get_logger(),
                            "Received TwistStamped with zero timestamp, setting it to current "
                            "time, this message will only be shown once");
                        msg->header.stamp = get_node()->get_clock()->now();
                    }
                    received_velocity_msg_ptr_.set(std::move(msg));
                });
        }
        else
        {
            velocity_command_unstamped_subscriber_ =
                get_node()->create_subscription<geometry_msgs::msg::Twist>(
                    "~/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS(),
                    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
                    {
                        if (!is_subscriber_active_)
                        {
                            RCLCPP_WARN(
                                get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                            return;
                        }
                        std::shared_ptr<Twist> twist_stamped;
                        received_velocity_msg_ptr_.get(twist_stamped);
                        twist_stamped->twist = *msg;
                        twist_stamped->header.stamp = get_node()->get_clock()->now();
                    });
        }
    }

} // namespace chassis_controllers

namespace chassis_controllers
{
    RollerWheel::RollerWheel() : ChassisBase::ChassisBase(){};
    controller_interface::CallbackReturn RollerWheel::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<roller_wheel::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        wheel_names_.push_back(params_.left_front_wheel_name);
        wheel_names_.push_back(params_.left_back_wheel_name);
        wheel_names_.push_back(params_.right_back_wheel_name);
        wheel_names_.push_back(params_.right_front_wheel_name);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RollerWheel::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        auto logger = get_node()->get_logger();
        if (param_listener_->is_old(params_))
        {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(logger, "Parameters were updated");
        }
        auto it = std::find_if(wheel_names_.begin(), wheel_names_.end(),
                               [](const std::string &wheel_name) -> bool
                               {
                                   return (wheel_name.empty());
                               });
        if (it != wheel_names_.end())
        {
            RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
            return controller_interface::CallbackReturn::ERROR;
        }

        ChassisBase::on_configure(previous_state);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RollerWheel::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const std::string &wheel_name : wheel_names_)
        {
            conf_names.push_back(wheel_name + "/" + hardware_interface::HW_IF_EFFORT);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration RollerWheel::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const std::string &wheel_name : wheel_names_)
        {
            conf_names.push_back(wheel_name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::CallbackReturn RollerWheel::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        const auto result = configure_joints(wheel_names_, registered_wheel_handles_);

        if (result == controller_interface::CallbackReturn::ERROR)
        {
            return controller_interface::CallbackReturn::ERROR;
        }
        is_subscriber_active_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "activate successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RollerWheel::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
        received_velocity_msg_ptr_.get(last_command_msg);

        geometry_msgs::msg::TwistStamped command = *last_command_msg;
        move_joints(command);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn RollerWheel::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RollerWheel::configure_joints(
        const std::vector<std::string> &wheel_names, std::vector<WheelHandle> &registered_handles)
    {
        auto logger = get_node()->get_logger();

        // register handles
        registered_handles.reserve(wheel_names.size());
        for (const auto &wheel_name : wheel_names)
        {
            const auto state_interface_type = hardware_interface::HW_IF_VELOCITY;
            const auto state_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(),
                [&wheel_name, &state_interface_type](const auto &interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                           interface.get_interface_name() == state_interface_type;
                });

            if (state_handle == state_interfaces_.cend())
            {
                RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            const auto command_interface_type = hardware_interface::HW_IF_EFFORT;
            const auto command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&wheel_name, &command_interface_type](const auto &interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                           interface.get_interface_name() == command_interface_type;
                });

            if (command_handle == command_interfaces_.end())
            {
                RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            registered_handles.emplace_back(
                WheelHandle{wheel_name, std::ref(*state_handle), std::ref(*command_handle)});
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn RollerWheel::move_joints(geometry_msgs::msg::TwistStamped &command)
    {
        double &linear_x_command = command.twist.linear.x;
        double &linear_y_command = command.twist.linear.y;
        double &angular_command = command.twist.angular.z;
        registered_wheel_handles_[0].eff_cmd.get().set_value(linear_x_command);
        registered_wheel_handles_[1].eff_cmd.get().set_value(-linear_x_command);
        registered_wheel_handles_[2].eff_cmd.get().set_value(linear_x_command);
        registered_wheel_handles_[3].eff_cmd.get().set_value(-linear_x_command);
    }
} // namespace chassis_controllers"

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    chassis_controllers::RollerWheel, controller_interface::ControllerInterface)