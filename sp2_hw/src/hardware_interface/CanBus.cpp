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

    /*!
     * @brief   constrain the number with limitiation.
     * @return  eg : limitAmplitude(-3000,1000) returns -1000
     * @todo    (Lithesh)move this function into utilities package
     */
    template <typename T>
    inline T limitAmplitude(T a, T limit)
    {
        limit = fabs(limit);
        int8_t sign = a < 0 ? -1 : 1;
        if (std::isnan(a))
            return 0.0;
        else if (fabs(a) < limit)
            return a;
        else
            return static_cast<float>(sign * limit);
    }

    CanBus::CanBus(const std::string &name, CanBusData can_bus_data)
        : bus_name_(name), can_bus_data_(can_bus_data)
    {
        socket_can_.configure(bus_name_, std::bind(&CanBus::recvCallback, this, std::placeholders::_1));
        using namespace std::chrono_literals;
        while ((!socket_can_.open()) && rclcpp::ok())
            rclcpp::sleep_for(1s);
        rm_frame_0x200_.can_id = 0x200;
        rm_frame_0x200_.can_dlc = 8;
        rm_frame_0x1FF_.can_id = 0x1FF;
        rm_frame_0x1FF_.can_dlc = 8;
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
                    // TODO 检查一下这个if是不是不要也可以
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
        bool has_write_frame0 = false, has_write_frame1 = false;
        std::fill(std::begin(rm_frame_0x1FF_.data), std::end(rm_frame_0x1FF_.data), 0);
        std::fill(std::begin(rm_frame_0x200_.data), std::end(rm_frame_0x200_.data), 0);

        for (auto &id2act_data : *can_bus_data_.id2act_data_)
        {
            // RM motors : id ranges from 0x201 to 0x208
            if (id2act_data.second.type.find("rm") != std::string::npos)
            {

                if (id2act_data.second.is_halted)
                    continue;
                const ActCoeff &act_coeff = can_bus_data_.type2act_coeff_->find(id2act_data.second.type)->second;
                int id = id2act_data.first - 0x201;
                double cmd =
                    limitAmplitude(act_coeff.eff2act * id2act_data.second.exe_cmd, act_coeff.max_out);
                if (-1 < id && id < 4)
                {
                    rm_frame_0x200_.data[2 * id] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                    rm_frame_0x200_.data[2 * id + 1] = static_cast<uint8_t>(cmd);
                    has_write_frame0 = true;
                }
                else if (3 < id && id < 8)
                {
                    rm_frame_0x1FF_.data[2 * (id - 4)] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                    rm_frame_0x1FF_.data[2 * (id - 4) + 1] = static_cast<uint8_t>(cmd);
                    has_write_frame1 = true;
                }
            }
        }
        if (has_write_frame0)
            socket_can_.write(&rm_frame_0x200_);
        if (has_write_frame1)
            socket_can_.write(&rm_frame_0x1FF_);
    }

    CanBus::~CanBus()
    {
        rm_frame_0x200_.can_id = 0x200;
        rm_frame_0x1FF_.can_id = 0x1FF;
        rm_frame_0x200_.can_dlc = 8;
        rm_frame_0x1FF_.can_dlc = 8;
        for (int i = 0; i < 8; ++i)
        {
            rm_frame_0x200_.data[i] = 0x00;
            rm_frame_0x1FF_.data[i] = 0x00;
        }
        socket_can_.write(&rm_frame_0x200_);
        socket_can_.write(&rm_frame_0x1FF_);
    }
    void CanBus::recvCallback(const can_frame &rx_frame)
    {
        /* 已是无锁队列，无需再加锁 */
        CanFrameStamp can_frame_stamp{.stamp = rclcpp::Clock().now(),
                                      .frame = rx_frame};
        rx_buffer_.push(can_frame_stamp);
    }

} // namespace SP2Control