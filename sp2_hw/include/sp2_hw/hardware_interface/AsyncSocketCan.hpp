/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, LitheshSari
 * All rights reserved.
 */

#pragma once

#include <iostream>
#include <thread>

#include <linux/can.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <boost/function.hpp>

constexpr int MAX_EVENTS = 10;

namespace can
{
    class SocketCan
    {
    public:
        int socket_fd_ = -1;
        int epoll_fd_ = -1;
        bool terminate_receiver_thread_ = false;
        bool receiver_thread_running_ = false;

        SocketCan() = default;
        ~SocketCan();

        /** \brief Open and bind socket.
         * \param interface bus's name(example: can0).
         * \param handler Pointer to a function which shall be called when frames are being received from the CAN bus.
         * \returns \c true if it successfully open and bind socket.
         */
        bool open(const std::string &interface, boost::function<void(const can_frame &frame)> handler);

        /** \brief Close and unbind socket.
         */
        void close();

        /** \brief Returns whether the socket is open or closed.
         * \returns \c True if socket has opened.
         */
        bool isOpen() const;
        /** \brief Sends the referenced frame to the bus.
         * \param frame referenced frame which you want to send.
         */
        void write(can_frame *frame) const;
        /**
         * Pointer to a function which shall be called
         * when frames are being received from the CAN bus
         */
        boost::function<void(const can_frame &frame)> reception_handler_;

    private:
        ifreq ifr_{};
        epoll_event events_[MAX_EVENTS];
        /** \brief receiver_thread
         */
        std::thread read_thread_{};
        void socketcan_receiver_thread(void);
    };
} // can