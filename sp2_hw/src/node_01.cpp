/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#include <iostream>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "sp2_hw/hardware_interface/AsyncUsart.hpp"
#include "sp2_hw/hardware_interface/CanBus.hpp"

void read_can(const can_frame &rx_frame)
{
    std::cout << "CAN ID: " << std::hex << int(rx_frame.can_id) << ", Data: ";
    for (int j = 0; j < rx_frame.can_dlc; ++j)
        std::cout << std::hex << std::setfill('0') << std::setw(2) << int(rx_frame.data[j]) << " ";
    std::cout << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_01");
    RCLCPP_INFO(node->get_logger(), "node_01 节点已经启动.");
    SP2Control::CanBusData can_bus_data;
    SP2Control::CanBus can_bus("can0", can_bus_data);

    // 创建串口读写线程
    // AsyncUsart usart_dbus;
    // std::thread thread(&AsyncUsart::serialThread, &usart_dbus, "/dev/ttyUSB0", 100000, std::ref(running));

    // ComBase的子类
    // SocketCan::SocketCan sock;
    // sock.configure("can0", read_can);
    // sock.open();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
