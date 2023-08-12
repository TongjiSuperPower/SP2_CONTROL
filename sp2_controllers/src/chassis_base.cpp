#include "sp2_controllers/chassis_base.hpp"
#include "diff_drive_controller_parameters.hpp"

namespace ChassisBase
{
    ChassisBase::ChassisBase() : controller_interface::ControllerInterface(){};

    controller_interface::CallbackReturn ChassisBase::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
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

} // namespace ChassisBase

namespace RollerWheel
{
    controller_interface::CallbackReturn RollerWheel::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RollerWheel::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        ChassisBase::on_configure(previous_state);
    }
} // namespace RollerWheel