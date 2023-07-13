#ifndef SRC_RM_BASE_INCLUDE_RT_RT_DBUS_H_
#define SRC_RM_BASE_INCLUDE_RT_RT_DBUS_H_

#include <iostream>
#include <cstring>
#include <string>
#include <unistd.h>

#include <thread>
#include <atomic>
#include <fcntl.h>

#include <sys/epoll.h>
#include <sys/ioctl.h>

// termbits.h and termios both contain struct<termios>
#define termios asmtermios
#include <asm-generic/termbits.h>
#undef termios

#include <termios.h>

#define MAX_EVENTS 10

typedef struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s0;
    uint8_t s1;
    int16_t wheel;

    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
    uint16_t key;

} DBusData_t;

class AsyncUsart
{
public:
    AsyncUsart() = default;
    ~AsyncUsart() = default;

    void read();
    void serialThread(const std::string &port, int baudrate, std::atomic<bool> &running);

private:
    DBusData_t d_bus_data_{};
    int port_;
    int16_t buff_[18]{};
    bool unpack();
    bool is_update_ = false;
    bool is_success = false;
};

#endif // SRC_RM_BRIDGE_INCLUDE_RT_RT_DBUS_H_