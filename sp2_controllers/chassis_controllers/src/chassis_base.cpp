#include "chassis_controllers/chassis_base.hpp"

namespace chassis_controllers
{
    ChassisBase::ChassisBase() : controller_interface::ControllerInterface() {}

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
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ChassisBase::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
        received_velocity_msg_ptr_.get(last_command_msg);

        geometry_msgs::msg::TwistStamped command = *last_command_msg;
        move_joints(command, period);
        update_pids();

        return controller_interface::return_type::OK;
    }
} // namespace chassis_controllers
