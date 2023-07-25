
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
         *  ROS2 Hardware并没有外层node的权限，因此只能使用generate_parameter_library(https://github.com/PickNikRobotics/generate_parameter_library)硬编码，
         *  后续如果有更新这部分可以重新写(https://github.com/ros-controls/ros2_control/issues/347)
         *  https://github.com/ros-controls/ros2_controllers/blob/master/forward_command_controller/include/forward_command_controller/forward_command_controller.hpp
         *  我真的不想写接下来的这段代码，但是ROS2不仅不支持全局parameter，而且目前似乎并没有类似于ROS1的XmlRpcValue数据
         *  类型，所以很难表达多层进的参数列表。
         *  希望的Feature如下：
         *  recursive_map = node->get_parameter("namespace");
         *  其中recursive_map是类多层哈希表结构，可以起到下述类似效果
         *  using RECURSIVE_MAP = unordered_map<std::string, RECURSIVE_MAP>
         */
        auto node_ = std::make_shared<rclcpp::Node>("actuator_coefficient");
        auto param_listener = std::make_shared<actuator_coefficient::ParamListener>(node_);
        actuator_coefficient::Params params = param_listener->get_params();

        setActCoeffMap("rm_2006", params.rm_2006, type2act_coeff_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SP2Hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SP2Hardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_state_));

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