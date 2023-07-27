/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */
#include "sp2_hw/hardware.hpp"

namespace SP2Control
{
    hardware_interface::CallbackReturn SP2Hardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        /**
         * @brief (Lithesh) 实现对于HardwareInfo内关节的解析
         * @todo  (Lithesh) 记得加try-block，因为可能URDF解析会出问题
         */
        const std::vector<hardware_interface::ComponentInfo> &joint_list = info.joints; // 为了可读性，就不自动推导了
        for (auto &joint : joint_list)
        {
            if (joint.parameters.find("bus_name") == joint.parameters.end())
            {
                // 假设URDF没有这一个键值对，就直接跳过对于该joint的解析
                std::cout << joint.name << " does not declare bus_name in URDF." << std::endl;
                continue;
            }
            const std::string &jnt_bus_name = joint.parameters.find("bus_name")->second;
            const std::string &jnt_type = joint.parameters.find("type")->second;
            const int jnt_id = std::stod(joint.parameters.find("ID")->second);
            // 如果还没有建立第一层哈希表，则建立
            if (bus_name2act_data_.find(jnt_bus_name) == bus_name2act_data_.end())
                bus_name2act_data_.emplace(make_pair(jnt_bus_name, ID2ACTDATA_MAP{}));
            bus_name2act_data_[jnt_bus_name].emplace(std::make_pair(
                jnt_id, ActData{
                            .name = joint.name,
                            .type = jnt_type,
                            .q_cur = 0,
                            .q_last = 0,
                            .qd_raw = 0,
                            .seq = 0,
                            .q_circle = 0,
                            .stamp = rclcpp::Clock().now(),
                            .offset = 0,
                            .pos = 0,
                            .vel = 0,
                            .acc = 0,
                            .eff = 0,
                            .exe_cmd = 0,
                            .cmd = 0,
                            .temperature = 25.,
                        }));
        }
        /**
         *  ROS2 Hardware并没有外层node的权限，因此只能使用generate_parameter_library(https://github.com/PickNikRobotics/generate_parameter_library)硬编码，
         *  后续如果有更新这部分可以重新写(https://github.com/ros-controls/ros2_control/issues/347)
         *  https://github.com/ros-controls/ros2_controllers/blob/master/forward_command_controller/include/forward_command_controller/forward_command_controller.hpp
         *  我真的不想写接下来的这段代码，但是ROS2不仅不支持全局parameter，而且目前似乎并没有类似于ROS1的XmlRpcValue数据类型，所以很难表达多层进的参数列表。
         *  希望的Feature如下：
         *  recursive_map = node->get_parameter("namespace");
         *  其中recursive_map是类多层哈希表结构，可以起到下述类似效果
         *  using RECURSIVE_MAP = unordered_map<std::string, RECURSIVE_MAP>
         */
        auto node_ = std::make_shared<rclcpp::Node>("actuator_coefficient");
        auto param_listener = std::make_shared<actuator_coefficient::ParamListener>(node_);
        actuator_coefficient::Params params = param_listener->get_params();
        setActCoeffMap("rm_2006", params.rm_2006, type2act_coeff_);
        setActCoeffMap("rm_3508", params.rm_3508, type2act_coeff_);

        for (auto &bus : bus_name2act_data_)
        {
            can_buses_.emplace_back(std::make_unique<CanBus>(bus.first,
                                                             CanBusData{
                                                                 .id2act_data_ = &bus.second,
                                                                 .type2act_coeff_ = &type2act_coeff_}));
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SP2Hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SP2Hardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto &id2act_data : bus_name2act_data_)
        {
            for (auto &act_data : id2act_data.second)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    act_data.second.name, hardware_interface::HW_IF_POSITION, &act_data.second.pos));
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    act_data.second.name, hardware_interface::HW_IF_VELOCITY, &act_data.second.vel));
            }
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SP2Hardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, hardware_interface::HW_IF_POSITION, &effort_command_));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn SP2Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SP2Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SP2Hardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (auto &can_bus : can_buses_)
            can_bus->read(rclcpp::Clock().now());
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SP2Hardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

    template <typename typeT>
    void SP2Hardware::setActCoeffMap(const std::string &type_name, const typeT &type, TYPE2ACTCOEFF_MAP &map)
    {
        // 真的太丑了，不知道在现在这个版本下有没有更好的写法
        ActCoeff act_coeff{};
        act_coeff.act2pos = type.act2pos;
        act_coeff.act2vel = type.act2vel;
        act_coeff.act2eff = type.act2eff;
        act_coeff.eff2act = type.eff2act;
        act_coeff.max_out = type.max_out;

        if (map.find(type_name) == map.end())
            map.emplace(make_pair(type_name, act_coeff));
    }

} // namespace SP2Control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(SP2Control::SP2Hardware, hardware_interface::SystemInterface)