/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */
/* (Lithesh)纪念并感谢我的奶奶 */

#pragma once

#include <iostream>
#include <thread>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <boost/function.hpp>
#include <atomic>

#define MAX_EVENTS 10

namespace ComBase
{
    /** \brief 通讯虚基类
     * \tparam ComProtocolT 通讯协议的数据帧数据类型，如<uint8_t>或<can_frame>
     */
    template <class ComProtocolT>
    class ComBase
    {
    public:
        // hardware构造时会调用各成员无参构造，因此要保留
        ComBase() = default;
        ComBase(const std::string &interface) : interface_name_(interface){};
        ComBase(const std::string &interface,
                boost::function<void(const ComProtocolT &rx_frame)> reception_handler)
            : interface_name_(interface), reception_handler_(std::move(reception_handler)){};
        ~ComBase();

        void setInterfaceName(const std::string &interface) { interface_name_ = interface; };
        // 回调函数传入方法
        void passRecptionHandler(boost::function<void(const ComProtocolT &rx_frame)> reception_handler)
        {
            reception_handler_ = std::move(reception_handler);
        };
        void write(ComProtocolT *rx_frame) const;

        bool open(void);
        // 只要有一者处在开启状态，就认为ComBase仍处于开启状态
        bool isOpen() { return (socket_fd_ != -1 || epoll_fd_ != -1 || receiver_thread_running_ == 1); };
        void close(void);

    protected:
        int socket_fd_ = -1;
        std::string interface_name_{};
        virtual bool openSocket(void) = 0;

    private:
        int epoll_fd_ = -1;
        epoll_event events_[MAX_EVENTS];
        std::atomic<bool> receiver_thread_running_{false};
        std::atomic<bool> terminate_receiver_thread_{false};

        bool openEpoll(void);
        void receiver_thread_(void);
        std::thread read_thread_{};
        boost::function<void(const ComProtocolT &rx_frame)> reception_handler_;
    };

    template <class ComProtocolT>
    inline ComBase<ComProtocolT>::~ComBase()
    {
        if (this->isOpen())
            this->close();
    }

    template <class ComProtocolT>
    inline void ComBase<ComProtocolT>::close(void)
    {
        terminate_receiver_thread_ = true;
        // RAII，防止线程异常导致程序崩溃
        if (read_thread_.joinable())
            read_thread_.join();

        // 关闭socket
        if (!this->isOpen())
            return;
        epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
        ::close(epoll_fd_);
        ::close(socket_fd_);
    }

    template <class ComProtocolT>
    inline bool ComBase<ComProtocolT>::open(void)
    {
        if (reception_handler_ == nullptr)
        {
            perror("Error reception_handler could not be empty");
            return false;
        }
        if (!(this->openSocket() && this->openEpoll()))
        {
            perror("Error creating communication");
            return false;
        }

        read_thread_ = std::thread(&ComBase::receiver_thread_, this);
        return true;
    }

    template <class ComProtocolT>
    inline bool ComBase<ComProtocolT>::openEpoll(void)
    {
        if (socket_fd_ == -1)
        {
            // 防止子类没有关闭socket
            ::close(socket_fd_);
            perror("Error using file descriptor");
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

        return true;
    }

    template <class ComProtocolT>
    inline void ComBase<ComProtocolT>::receiver_thread_(void)
    {
        ComProtocolT rx_frame{};
        receiver_thread_running_ = true;
        while (!terminate_receiver_thread_)
        {
            // 用events存储发生了事件的file descriptor
            int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, -1);
            if (num_events == -1)
            {
                perror("Error while waiting for events");
                return;
            }
            // 实际上num_events肯定等于1，因为只申请了一个File Descriptor
            for (int i = 0; i < num_events; ++i)
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
        receiver_thread_running_ = false;
    }

    template <class ComProtocolT>
    inline void ComBase<ComProtocolT>::write(ComProtocolT *tx_frame) const
    {
        if (!isOpen())
        {
            printf("Can not write, %s not opened.", interface_name_);
            return;
        }
        if (::write(socket_fd_, tx_frame, sizeof(ComProtocolT)) == -1)
        {
            std::string error(interface_name_ + " unable to write");
            perror(error.c_str());
        }
    }
} // namespace ComBase