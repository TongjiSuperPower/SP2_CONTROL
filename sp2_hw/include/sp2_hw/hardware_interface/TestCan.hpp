#include "sp2_hw/hardware_interface/ComBase.hpp"
#include <linux/can.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <boost/function.hpp>

class TestCan final : public ComBase::ComBase<can_frame>
{
public:
    TestCan() = default;
    TestCan(const std::string &interface) : ComBase(interface){};

private:
    ifreq ifr_{};
    virtual bool openSocket(void) override;
};

bool TestCan::openSocket(void)
{ // Open socketCan
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        perror("Error opening socket");
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