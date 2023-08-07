/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */
#include "sp2_hw/hardware.hpp"

namespace SP2Control
{
    void plot(const std::unordered_map<std::string, ActCoeff> &map)
    {
        for (auto it = map.begin(); it != map.end(); ++it)
        {
            std::cout << it->first << std::endl;
            std::cout << "    "
                      << "act2pos: " << it->second.act2pos << std::endl;
        }
    }
    hardware_interface::CallbackReturn SP2Hardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        /**
         * @brief (Lithesh) 实现对于HardwareInfo内关节的解析
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
                jnt_id, ActData(joint.name, jnt_type, rclcpp::Clock().now())));
            // 为了方便transmission的查询，建立<joint_name, jnt_data>哈希表
            jnt_name2jnt_data_ptr_.emplace(joint.name, &(bus_name2act_data_[jnt_bus_name][jnt_id]));
        }

        /**
         *  基于内存和运行速度上的考虑，不需要为没有transmssion的actuator设定JntData，只需要复用ActData的数据即可。
         *  并且考虑到目前transmission的设计思路，可以为每个带transmission的关节实例化两个transmission，
         *  也即jnt2act_transmission_和act2jnt_transmission_，前者负责状态量的转化，后者负责控制量的转化。
         *  目前只支持simple类型的Transmission
         */
        auto simple_transmission_loader = transmission_interface::SimpleTransmissionLoader();
        //  先遍历所有的transmission
        for (const auto &transmission_info : info_.transmissions)
        {
            if (transmission_info.type != "transmission_interface/SimpleTransmission")
            {
                printf("Transmission '%s' of type '%s' not supported.\n",
                       transmission_info.name.c_str(), transmission_info.type.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            std::shared_ptr<transmission_interface::Transmission> act2jnt_transmission;
            std::shared_ptr<transmission_interface::Transmission> jnt2act_transmission;
            //  由于可能有参数错误或缺失的问题，因此要有try-block块
            try
            {
                act2jnt_transmission = simple_transmission_loader.load(transmission_info);
                jnt2act_transmission = simple_transmission_loader.load(transmission_info);
            }
            catch (const transmission_interface::TransmissionInterfaceException &e)
            {
                printf("Error while loading transmission %s: %s", transmission_info.name.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }

            std::vector<transmission_interface::ActuatorHandle> actuator_state_handles;
            std::vector<transmission_interface::JointHandle> joint_state_handles;
            std::vector<transmission_interface::ActuatorHandle> actuator_cmd_handles;
            std::vector<transmission_interface::JointHandle> joint_cmd_handles;
            //  开始遍历每个transmission内部的joint容器，但由于是simple型，因此其size只能为1
            for (const auto &joint_info : transmission_info.joints)
            {
                auto it = jnt_name2jnt_data_ptr_.find(joint_info.name);
                if (it == jnt_name2jnt_data_ptr_.end())
                {
                    printf("Error while setting up transmission, while '%s' isn't declared in URDF", joint_info.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                const auto jnt_data_it = jnt_data_.insert(jnt_data_.end(), JntData(joint_info.name));
                //  Transmission源码暂时不支持accerleration
                actuator_state_handles.push_back(transmission_interface::ActuatorHandle(
                    joint_info.name, hardware_interface::HW_IF_POSITION, &(it->second->pos)));
                actuator_state_handles.push_back(transmission_interface::ActuatorHandle(
                    joint_info.name, hardware_interface::HW_IF_VELOCITY, &(it->second->vel)));
                actuator_state_handles.push_back(transmission_interface::ActuatorHandle(
                    joint_info.name, hardware_interface::HW_IF_EFFORT, &(it->second->eff)));
                joint_state_handles.push_back(transmission_interface::JointHandle(
                    joint_info.name, hardware_interface::HW_IF_POSITION, &(jnt_data_it->pos)));
                joint_state_handles.push_back(transmission_interface::JointHandle(
                    joint_info.name, hardware_interface::HW_IF_VELOCITY, &(jnt_data_it->vel)));
                joint_state_handles.push_back(transmission_interface::JointHandle(
                    joint_info.name, hardware_interface::HW_IF_EFFORT, &(jnt_data_it->eff)));

                // TODO 根据其command_interface选择使用哪种interface，现在只能用HW_IF_EFFORT
                actuator_cmd_handles.push_back(transmission_interface::ActuatorHandle(
                    joint_info.name, hardware_interface::HW_IF_EFFORT, &(it->second->cmd)));
                joint_cmd_handles.push_back(transmission_interface::JointHandle(
                    joint_info.name, hardware_interface::HW_IF_EFFORT, &(it->second->cmd)));
                /**
                 * 若Joint有Transmission，则使用新建立的JntData替换掉jnt_name2jnt_data_ptr中的ActData
                 * 从迭代器获得左值，然后再取其地址。虽然测试了简单类型，但还是感觉这个操作有点点不安全。
                 */
                it->second = &(*jnt_data_it);

                try
                {
                    act2jnt_transmission->configure(joint_state_handles, actuator_state_handles);
                    jnt2act_transmission->configure(joint_cmd_handles, actuator_cmd_handles);
                }
                catch (const transmission_interface::TransmissionInterfaceException &e)
                {
                    printf("Error while configuring transmission %s: %s", transmission_info.name.c_str(), e.what());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                act2jnt_transmissions_.push_back(act2jnt_transmission);
                jnt2act_transmissions_.push_back(jnt2act_transmission);
            }
        }
        /**
         *  ROS2 Hardware并没有外层node的权限，因此只能使用generate_parameter_library(https://github.com/PickNikRobotics/generate_parameter_library)硬编码，
         *  后续如果有更新这部分可以重新写(https://github.com/ros-controls/ros2_control/issues/347)
         *  我真的不想写接下来的这段代码，但是ROS2不仅不支持全局parameter，而且目前似乎并没有类似于ROS1的XmlRpcValue数据类型，所以很难表达多层进的参数列表。
         *  希望的Feature如下：
         *  recursive_map = node->get_parameter("namespace");
         *  其中recursive_map是类多层哈希表结构，可以起到下述类似效果
         *  using RECURSIVE_MAP = unordered_map<std::string, RECURSIVE_MAP>
         *  (Lithesh 20230803): 找到了一种比较优雅的方法。
         */
        YAML::Node config;
        try
        {
            std::string PKG_SHARE_PATH(PROJECT_SHARE_PATH);
            config = YAML::LoadFile(PKG_SHARE_PATH + "/share/sp2_hw/config/actuator_coefficient.yaml");
        }
        catch (YAML::BadFile &e)
        {
            std::cout << "Error reading actuator coefficient YAML file" << std::endl;
            return hardware_interface::CallbackReturn::ERROR;
        }
        const YAML::Node &actuator_coefficient_map = config["actuator_coefficient"];
        for (YAML::const_iterator it = actuator_coefficient_map.begin(); it != actuator_coefficient_map.end(); ++it)
        {
            auto &param_list = it->second;
            std::vector<std::string> param_names{"act2pos", "act2vel", "act2eff", "eff2act", "max_out"};
            bool isErrorExists = false;
            for (const std::string &param_name : param_names)
            {
                if (!param_list[param_name].IsDefined())
                {
                    printf("motor %s missing parameter: %s", it->first.as<std::string>().c_str(), param_name.c_str());
                    isErrorExists = true;
                }
            }
            if (isErrorExists)
                continue;
            type2act_coeff_.emplace(std::make_pair(it->first.as<std::string>(),
                                                   ActCoeff{
                                                       .act2pos = param_list["act2pos"].as<double>(),
                                                       .act2vel = param_list["act2vel"].as<double>(),
                                                       .act2eff = param_list["act2eff"].as<double>(),
                                                       .pos2act = (1.0 / param_list["act2pos"].as<double>()),
                                                       .vel2act = (1.0 / param_list["act2vel"].as<double>()),
                                                       .eff2act = param_list["eff2act"].as<double>(),
                                                       .max_out = param_list["max_out"].as<double>(),
                                                   }));
        }

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
        for (auto &jnt_data_ptr_pair : jnt_name2jnt_data_ptr_)
        {
            auto &jnt_data_ptr = jnt_data_ptr_pair.second;
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                jnt_data_ptr->name, hardware_interface::HW_IF_POSITION, &jnt_data_ptr->pos));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                jnt_data_ptr->name, hardware_interface::HW_IF_VELOCITY, &jnt_data_ptr->vel));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                jnt_data_ptr->name, hardware_interface::HW_IF_EFFORT, &jnt_data_ptr->eff));
        }

        // for (auto &id2act_data : bus_name2act_data_)
        // {
        //     for (auto &act_data : id2act_data.second)
        //     {
        //         state_interfaces.emplace_back(hardware_interface::StateInterface(
        //             act_data.second.name, hardware_interface::HW_IF_POSITION, &act_data.second.pos));
        //         state_interfaces.emplace_back(hardware_interface::StateInterface(
        //             act_data.second.name, hardware_interface::HW_IF_VELOCITY, &act_data.second.vel));
        //         state_interfaces.emplace_back(hardware_interface::StateInterface(
        //             act_data.second.name, hardware_interface::HW_IF_EFFORT, &act_data.second.eff));
        //     }
        // }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SP2Hardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto &id2act_data : bus_name2act_data_)
        {
            for (auto &act_data : id2act_data.second)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    act_data.second.name, hardware_interface::HW_IF_EFFORT, &act_data.second.exe_cmd));
            }
        }
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
        std::for_each(
            act2jnt_transmissions_.begin(), act2jnt_transmissions_.end(),
            [](auto &transmission)
            { transmission->actuator_to_joint(); });
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SP2Hardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        std::for_each(
            jnt2act_transmissions_.begin(), jnt2act_transmissions_.end(),
            [](auto &transmission)
            { transmission->joint_to_actuator(); });
        /*------------------------------- IN_TEST --------------------------------*/
        ActData *p = static_cast<ActData *>(jnt_name2jnt_data_ptr_["arm_joint"]);
        p->exe_cmd = 10 * (-p->pos) + 0.4 * (-p->vel);
        /*------------------------------- IN_TEST --------------------------------*/
        for (auto &can_bus : can_buses_)
            can_bus->write();

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