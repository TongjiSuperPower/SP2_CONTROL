/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#include "sp2_hw/hardware_interface/CanBus.hpp"

namespace SP2Control
{
    const uint8_t kReadCountMax = 20;

    CanBus::CanBus(const std::string &name, CanBusData can_bus_data)
        : bus_name_(name), can_bus_data_(can_bus_data)
    {
        socket_can_.configure(bus_name_, std::bind(&CanBus::recvCallback, this, std::placeholders::_1));
        using namespace std::chrono_literals;
        while ((!socket_can_.open()) && rclcpp::ok())
            rclcpp::sleep_for(1s);

        std::cout << "CanBus " << bus_name_ << " constructed successfully" << std::endl;
    }
    void CanBus::read(const rclcpp::Time &time)
    {
        CanFrameStamp can_frame_stamp;
        /**
         * 循环解析无锁队列
         * (Lithesh)有点担心采用无锁队列后由于IO读写过快，导致while循环无法退出
         */
        uint8_t read_cout = 0;
        while (rx_buffer_.pop(can_frame_stamp) && (read_cout < kReadCountMax))
        {
            can_frame frame = can_frame_stamp.frame;
            // Check if the motor has been registered via ID
            if (can_bus_data_.id2act_data_->find(frame.can_id) != can_bus_data_.id2act_data_->end())
            {
                ActData &act_data = can_bus_data_.id2act_data_->find(frame.can_id)->second;
                const ActCoeff &act_coeff = can_bus_data_.type2act_coeff_->find(act_data.type)->second;
                if (act_data.type.find("rm") != std::string::npos)
                {
                    act_data.q_last = act_data.q_cur;
                    act_data.q_cur = (frame.data[0] << 8u) | frame.data[1];
                    act_data.qd_raw = (frame.data[2] << 8u) | frame.data[3];
                    int16_t mapped_current = (frame.data[4] << 8u) | frame.data[5];
                    act_data.stamp = can_frame_stamp.stamp;

                    // 防止上电后圈数计算多一圈或者少一圈
                    if (act_data.seq < 10)
                        act_data.q_last = act_data.q_cur;

                    if (act_data.seq != 0)
                    {
                        // Basically, motor won't rotate more than 4096 between two time slide.
                        if (act_data.q_cur - act_data.q_last > 4096)
                            act_data.q_circle--;
                        else if (act_data.q_last - act_data.q_cur > 4096)
                            act_data.q_circle++;
                    }
                    act_data.seq++;
                    // convert raw data into standard ActuatorState
                    act_data.pos = act_coeff.act2pos *
                                   static_cast<double>(act_data.q_cur + 8192 * act_data.q_circle - act_data.offset);

                    act_data.pos = act_data.pos / 19;
                    act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
                    act_data.eff = act_coeff.act2eff * static_cast<double>(mapped_current);
                    // std::cout << "vel: " << std::setw(10) << act_data.vel
                    //           << std::setw(10) << "pos: " << act_data.pos << std::endl;
                    read_cout++;
                    continue;
                }
            }
            /*
            else if(){
            }
            */
        }
    }

    void CanBus::write()
    {
        ;
    }

    void CanBus::recvCallback(const can_frame &rx_frame)
    {
        // std::cout << "CAN ID: " << std::hex << int(rx_frame.can_id) << ", Data: ";
        // for (int j = 0; j < rx_frame.can_dlc; ++j)
        // std::cout << std::hex << std::setfill('0') << std::setw(2) << int(rx_frame.data[j]) << " ";
        // std::cout << std::endl;
        /* 已是无锁队列，无需再加锁 */
        CanFrameStamp can_frame_stamp{.stamp = rclcpp::Clock().now(),
                                      .frame = rx_frame};
        rx_buffer_.push(can_frame_stamp);
    }

} // namespace SP2Control