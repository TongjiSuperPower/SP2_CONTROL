/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */
#pragma once

#include "sp2_hw/hardware_interface/SocketCan.hpp"
#include <unordered_map>

namespace SP2Control
{
    struct ActData
    {
        std::string name;
        std::string type;
        uint16_t q_cur;
        uint16_t q_last;
        uint16_t qd_raw;
        // 一般不会溢出
        int64_t seq;
        int64_t q_circle;

        double offset;
        double pos, vel, acc;
        // (Lithesh)TODO: 可能存在非力控的电机协议
        /**
         * @param exe_cmd 为最终执行指令
         * @param cmd 为未加可能额外限制的指令
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

    class CanBus
    {
        using ID2ACTDATA_MAP = std::unordered_map<int, ActData>;
        using TYPE2ACTCOEFF_MAP = std::unordered_map<std::string, ActCoeff>;

    public:
        CanBus() = delete;
        CanBus(const std::string &name, ID2ACTDATA_MAP *id2act_data, TYPE2ACTCOEFF_MAP *type2act_coeff);
        void read();
        void write();

    private:
        SocketCan::SocketCan socket_can_;
        void recvCallback(const can_frame &rx_frame);
        CanBusData can_bus_data_;
    };

} // namespace SP2Control