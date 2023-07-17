/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#include "sp2_hw/hardware_interface/ComBase.hpp"
#include <linux/can.h>
#include <net/if.h>
#include <boost/function.hpp>
#include <functional>
#include <iomanip>

namespace SocketCan
{
    class SocketCan final : public ComBase::ComBase<can_frame>
    {
    public:
        SocketCan() = delete;
        SocketCan(const std::string &interface) : ComBase<can_frame>(interface)
        {
            passRecptionHandler(std::bind(&SocketCan::read_can, this, std::placeholders::_1));
        };
        SocketCan(const std::string &interface,
                  boost::function<void(const can_frame &rx_frame)> reception_handler)
            : ComBase<can_frame>(interface, reception_handler){};

        void read_can(const can_frame &rx_frame);

    private:
        ifreq ifr_{};
        virtual bool openSocket(void) override;
    };

    void SocketCan::read_can(const can_frame &rx_frame)
    {
        std::cout << "CAN ID: " << std::hex << int(rx_frame.can_id) << ", Data: ";
        for (int j = 0; j < rx_frame.can_dlc; ++j)
            std::cout << std::hex << std::setfill('0') << std::setw(2) << int(rx_frame.data[j]) << " ";
        std::cout << std::endl;
    }

    bool SocketCan::openSocket(void)
    { // Open socketCan
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0)
        {
            perror("Error opening socket");
            close();
            return false;
        }

        if (interface_name_.empty())
        {
            perror("Error interface_name could not be emtpy");
            close();
            return false;
        }
        // Bind the socket to the CAN interface
        std::strncpy(ifr_.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);

        // 查找设备索引
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr_) < 0)
        {
            perror("Error getting interface index");
            close();
            return false;
        }
        sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr_.ifr_ifindex;

        if (bind(socket_fd_, (sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("Error binding socket to interface");
            ::close(socket_fd_);
            return false;
        }
        return true;
    }
} // namespace SocketCan