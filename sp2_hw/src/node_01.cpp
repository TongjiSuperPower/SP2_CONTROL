/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#include <iostream>
#include <iomanip>
#include "sp2_hw/hardware_interface/AsyncSocketCan.hpp"
#include "sp2_hw/hardware_interface/AsyncUsart.hpp"
#include "sp2_hw/hardware_interface/TestCan.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_01");
    RCLCPP_INFO(node->get_logger(), "node_01 节点已经启动.");

    std::atomic<bool> running(true);

    // 创建串口读写线程
    AsyncUsart usart_dbus;
    std::thread thread(&AsyncUsart::serialThread, &usart_dbus, "/dev/ttyUSB0", 100000, std::ref(running));

    SocketCan::SocketCan sock("can0");
    sock.open();
    // can::SocketCan sock;
    // sock.open("can0", read_can);

    rclcpp::spin(node);
    rclcpp::shutdown();
    running = false;
    return 0;
}
