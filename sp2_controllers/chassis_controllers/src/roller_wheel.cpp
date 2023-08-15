#include "chassis_controllers/roller_wheel.hpp"

namespace chassis_controllers
{
    RollerWheel::RollerWheel() : ChassisBase::ChassisBase(){};
    controller_interface::CallbackReturn RollerWheel::on_init()
    {
        nh_ = get_node();
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

        dof_ = params_.joints.size();
        wheel_names_.reserve(dof_);
        for (auto &joint : params_.joints)
        {
            wheel_names_.emplace_back(joint);
        }

        parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            get_node()->get_node_base_interface(),
            get_node()->get_node_topics_interface(),
            get_node()->get_node_graph_interface(),
            get_node()->get_node_services_interface());
        parameter_event_sub_ = parameters_client_->on_parameter_event(
            std::bind(&RollerWheel::on_parameter_event_callback, this, std::placeholders::_1));

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

        wheel_base_ = params_.wheel_base;
        wheel_track_ = params_.wheel_track;
        wheel_radius_ = params_.wheel_radius;

        // Init PID gains from ROS parameters
        pids_.resize(dof_);
        for (size_t i = 0; i < dof_; ++i)
        {
            try
            {
                std::string joint_name = wheel_names_[i];
                auto &gain = params_.gains.joints_map.at(joint_name);
                pids_[i] = std::make_shared<control_toolbox::Pid>(
                    gain.p, gain.i, gain.d, gain.i_clamp, -gain.i_clamp);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        wheel_vel_command_.resize(dof_, 0.0);
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

    controller_interface::CallbackReturn RollerWheel::move_joints(
        geometry_msgs::msg::TwistStamped &command, const rclcpp::Duration &period)
    {
        geometry_msgs::msg::Vector3 vel_cmd_;
        std::vector<double> wheel_vel_target(4);
        std::vector<double> wheel_vel_state(4);
        vel_cmd_.x = command.twist.linear.x;
        vel_cmd_.y = command.twist.linear.y;
        vel_cmd_.z = command.twist.angular.z;

        double a = (wheel_base_ + wheel_track_) / 2.0;
        wheel_vel_target[0] = ((vel_cmd_.x - vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        wheel_vel_target[1] = ((vel_cmd_.x + vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        wheel_vel_target[2] = ((vel_cmd_.x - vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        wheel_vel_target[3] = ((vel_cmd_.x + vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);

        for (size_t i = 0; i < dof_; ++i)
        {
            wheel_vel_state[i] = registered_wheel_handles_[i].vel_state.get().get_value();
            double error = wheel_vel_target[i] - wheel_vel_state[i];
            wheel_vel_command_[i] = pids_[i]->computeCommand(error, period.nanoseconds());
            registered_wheel_handles_[i].eff_cmd.get().set_value(wheel_vel_command_[i]);
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void RollerWheel::update_pids()
    {
        if (!configured_pid_gains_.empty())
        {
            std::unordered_map<std::string, int> joint_name2index;
            for (size_t i = 0; i < dof_; ++i)
                joint_name2index.emplace(wheel_names_[i], i);
            auto logger = get_node()->get_logger();
            params_ = param_listener_->get_params();
            RCLCPP_INFO(logger, "Parameters were updated");
            for (auto &pid_gains : configured_pid_gains_)
            {
                std::string joint_name = pid_gains.first;
                size_t index = joint_name2index[joint_name];
                auto &gain = params_.gains.joints_map.at(joint_name);
                pids_[index]->setGains(gain.p, gain.i, gain.d, gain.i_clamp, -gain.i_clamp);
            }
            configured_pid_gains_.clear();
        }
    }

    void RollerWheel::on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {
        for (auto &changed_parameter : event->changed_parameters)
        {
            const std::string &name = changed_parameter.name;

            size_t array_index[2];
            size_t string_end = name.size();
            for (int i = 0; i < 2; ++i)
            {
                size_t index = name.rfind('.', string_end);
                if (index == std::string::npos)
                    return;
                string_end = index - 1;
                array_index[1 - i] = index;
            }

            std::string joint_name = name.substr(array_index[0] + 1, array_index[1] - array_index[0] - 1);
            auto it = configured_pid_gains_.find(joint_name);
            if (it == configured_pid_gains_.end())
            {
                configured_pid_gains_.emplace(std::make_pair(joint_name, true));
            }
        }
    }
} // namespace chassis_controllers"

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    chassis_controllers::RollerWheel, controller_interface::ControllerInterface)