/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#include "sp2_hw/hardware_interface/CanBus.hpp"
namespace SP2Control
{
    CanBus::CanBus(const std::string &name, CanBusData can_bus_data)
        : can_bus_data_(can_bus_data), socket_can_(name, recvCallback), bus_name_(name)
    {
        while (!socket_can_.open())
            sleep(5);

        std::cout << "CanBus " << bus_name_ << " constructed successfully" << std::endl;
    }
    void CanBus::read()
    {
        CanFrameStamp can_frame_stamp;
        // 循环解析无锁队列
        while (rx_buffer_.pop(can_frame_stamp))
        {
            ;
        }
    }

    void CanBus::write()
    {
    }

    void CanBus::recvCallback(const can_frame &rx_frame)
    {
        // 已是无锁队列，无需再加锁
        CanFrameStamp can_frame_stamp{.stamp = rclcpp::Clock().now(),
                                      .frame = rx_frame};
        rx_buffer_.push(can_frame_stamp);
    }

} // namespace SP2Control