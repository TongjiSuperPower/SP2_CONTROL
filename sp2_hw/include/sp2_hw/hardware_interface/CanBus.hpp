/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#ifndef SP2_CONTROL_CAN_BUS_HPP
#define SP2_CONTROL_CAN_BUS_HPP

#include "sp2_hw/hardware_interface/SocketCan.hpp"
#include <unordered_map>
#include <boost/lockfree/spsc_queue.hpp>
#include <rclcpp/rclcpp.hpp>
#include "actuator_coefficient_lib.hpp"

namespace SP2Control
{
    struct ActData
    {
        std::string name;
        std::string type;
        uint16_t q_cur;
        uint16_t q_last;
        int16_t qd_raw;
        // 一般不会溢出
        int64_t seq;
        int64_t q_circle;
        rclcpp::Time stamp;

        double offset;
        double pos, vel, acc;
        double eff;
        // (Lithesh)TODO: 可能存在非力控的电机协议
        /**
         * @param  exe_cmd  最终执行指令
         * @param  cmd      未加可能额外限制的指令
         */
        double exe_cmd, cmd;
        bool is_halted = false;

        double temperature;
    };

    struct ActCoeff
    {
        double act2pos, act2vel, act2eff;
        double pos2act, vel2cat, eff2act;
        double max_out;
    };

    /**
     * @brief  \c CanBusData 包含了执行器数据哈希表的地址和电机参数哈希表的地址
     */
    struct CanBusData
    {
        std::unordered_map<int, ActData> *id2act_data_;
        std::unordered_map<std::string, ActCoeff> *type2act_coeff_;
    };

    struct CanFrameStamp
    {
        rclcpp::Time stamp;
        can_frame frame;
    };

    using ID2ACTDATA_MAP = std::unordered_map<int, ActData>;
    using TYPE2ACTCOEFF_MAP = std::unordered_map<std::string, ActCoeff>;

    /**
     * @brief 一个用于描述can总线的类。该类实现了 \c read() 和 \c write() 方法。
     * @brief 根据电机协议对数据解包的工作在该类中完成。
     */
    class CanBus
    {
    public:
        CanBus() = default;
        CanBus(const std::string &name, CanBusData can_bus_data);

        void read();
        void write();

    private:
        std::string bus_name_;

        CanBusData can_bus_data_;
        SocketCan::SocketCan socket_can_{};

        can_frame rm_frame_0x200;
        can_frame rm_frame_0x1FF;

        boost::lockfree::spsc_queue<CanFrameStamp> rx_buffer_{20};
        void recvCallback(const can_frame &rx_frame);
    };

} // namespace SP2Control
#endif