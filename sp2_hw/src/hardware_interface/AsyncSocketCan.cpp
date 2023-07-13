/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, LitheshSari
 * All rights reserved.
 */

#include "sp2_hw/hardware_interface/AsyncSocketCan.hpp"

namespace can
{
    SocketCan::~SocketCan()
    {
        if (this->isOpen())
            this->close();
    }

    void SocketCan::close()
    {
        terminate_receiver_thread_ = true;
        // 通过析构回收线程资源
        read_thread_.join();

        // 关闭socket
        if (!isOpen())
            return;
        epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
        ::close(epoll_fd_);
        ::close(socket_fd_);
    }

    bool SocketCan::isOpen() const
    {
        return (socket_fd_ != -1);
    }

    void SocketCan::socketcan_receiver_thread(void)
    {
        // How long 'select' shall wait before returning with timeout
        // struct timeval timeout
        //{
        //};
        // Buffer to store incoming frame
        can_frame rx_frame{};
        // Run until termination signal received
        receiver_thread_running_ = true;
        while (!terminate_receiver_thread_)
        {
            // 用events存储发生事件的fd
            int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, -1);
            if (num_events == -1)
            {
                perror("Error wating for events");
                return;
            }
            for (int i = 0; i < num_events; ++i)
            {
                // 只是为了保险，实际该判断应该永远为True
                if (events_[i].data.fd == socket_fd_)
                {
                    ssize_t num_bytes = recv(socket_fd_, &rx_frame, sizeof(rx_frame), MSG_DONTWAIT);
                    if (num_bytes == -1)
                    {
                        perror("Error reading from SocketCan");
                        return;
                    }
                    reception_handler_(rx_frame);
                }
            }
        }
        receiver_thread_running_ = false;
        return;
    }

    bool SocketCan::open(const std::string &interface, boost::function<void(const can_frame &frame)> handler)
    {
        reception_handler_ = std::move(handler);

        // Open socketCan
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0)
        {
            perror("Error opening socket");
            close();
            return false;
        }

        // Bind the socket to the CAN interface
        std::strncpy(ifr_.ifr_name, interface.c_str(), IFNAMSIZ - 1);

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

        epoll_event ev;
        epoll_fd_ = epoll_create1(0);
        if (epoll_fd_ == -1)
        {
            perror("Error creating epoll file descriptor");
            return false;
        }
        // 填充需检测事件描述
        ev.events = EPOLLIN;
        ev.data.fd = socket_fd_;
        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev))
        {
            perror("Error adding socket to epoll file descriptor");
            return false;
        }

        // Multithread
        read_thread_ = std::thread(&SocketCan::socketcan_receiver_thread, this);
        return true;
    }
}