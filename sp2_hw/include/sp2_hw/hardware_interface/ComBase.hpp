#pragma once

#include <iostream>
#include <thread>

#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <boost/function.hpp>
#include <atomic>

namespace ComBase
{
    class ComBase
    {
    public:
        ComBase() = default;
        ~ComBase();
        std::atomic<bool> receiver_thread_running_ = false;
        std::atomic<bool> terminate_receiver_thread_ = false;

    private:
    };
} // ComBase