/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#include "rclcpp/rclcpp.hpp"
#include "sp2_hw/hardware_interface/AsyncUsart.hpp"
#include "sp2_hw/hardware_interface/CanBus.hpp"

// void read_can(const can_frame &rx_frame)
//{
// std::cout << "CAN ID: " << std::hex << int(rx_frame.can_id) << ", Data: ";
// for (int j = 0; j < rx_frame.can_dlc; ++j)
// std::cout << std::hex << std::setfill('0') << std::setw(2) << int(rx_frame.data[j]) << " ";
// std::cout << std::endl;
//}

namespace SP2Control
{
    template <typename typeT>
    void setActCoeffMap(const std::string &type_name, const typeT &type, TYPE2ACTCOEFF_MAP &map)
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
    };
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_01");
    RCLCPP_INFO(node->get_logger(), "sp2_hw 测试节点已经启动.");

    //------------------------------IN___TEST___---------------------------------//
    SP2Control::TYPE2ACTCOEFF_MAP type2act_coeff;
    SP2Control::ID2ACTDATA_MAP id2act_data;
    id2act_data.emplace(std::make_pair(0x201, SP2Control::ActData{
                                                  .name = "joint1_motor",
                                                  .type = "rm_2006",
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
    auto node_ = std::make_shared<rclcpp::Node>("actuator_coefficient");
    auto param_listener = std::make_shared<actuator_coefficient::ParamListener>(node_);
    auto params = param_listener->get_params();

    SP2Control::setActCoeffMap("rm_2006", params.rm_2006, type2act_coeff);

    SP2Control::CanBusData can_bus_data{.id2act_data_ = &id2act_data, .type2act_coeff_ = &type2act_coeff};
    SP2Control::CanBus can_bus("can0", can_bus_data);
    //------------------------------IN___TEST___---------------------------------//

    while (rclcpp::ok())
        can_bus.read(rclcpp::Clock().now());

    // 创建串口读写线程
    // AsyncUsart usart_dbus;
    // std::thread thread(&AsyncUsart::serialThread, &usart_dbus, "/dev/ttyUSB0", 100000, std::ref(running));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
