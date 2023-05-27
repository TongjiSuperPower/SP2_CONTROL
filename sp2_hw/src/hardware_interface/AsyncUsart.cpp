#include "sp2_hw/hardware_interface/AsyncUsart.hpp"

// 串口读写线程函数
void AsyncUsart::serialThread(const std::string &port, int baudrate, std::atomic<bool> &running)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1)
    {
        std::cerr << "Failed to open serial port" << std::endl;
        return;
    }

    port_ = fd;

    // 配置串口
    struct termios2 options;
    if (ioctl(fd, TCGETS2, &options) == -1)
    {
        close(fd);
        fd = -1;
        perror("ioctl failed");
        return;
    }
    options.c_cflag &= ~CBAUD;
    options.c_cflag |= BOTHER;

    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_ispeed = baudrate;
    options.c_ospeed = baudrate;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~IGNBRK; // disable break processing

    /* set input mode (non−canonical, no echo,...) */
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    options.c_oflag = 0;                 // no remapping, no delays
    options.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    ioctl(fd, TCSETS2, &options);

    // 创建 epoll 实例
    int epoll_fd = epoll_create1(0);
    if (epoll_fd == -1)
    {
        std::cerr << "Failed to create epoll instance" << std::endl;
        close(fd);
        return;
    }

    // 添加串口文件描述符到 epoll
    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev) == -1)
    {
        std::cerr << "Failed to add serial port to epoll" << std::endl;
        close(fd);
        close(epoll_fd);
        return;
    }

    // 创建事件数组
    struct epoll_event events[MAX_EVENTS];

    while (running)
    {
        // 等待事件
        int event_count = epoll_wait(epoll_fd, events, MAX_EVENTS, -1);
        if (event_count == -1)
        {
            std::cerr << "Error in epoll_wait" << std::endl;
            break;
        }

        // 处理事件
        for (int i = 0; i < event_count; ++i)
        {
            if (events[i].events & EPOLLIN)
            {
                // 读取数据
                read();
            }
        }
    }

    // 关闭串口和 epoll
    close(fd);
    close(epoll_fd);
}
void AsyncUsart::read()
{
    uint8_t read_byte;
    int timeout = 0; // time out of one package
    int count = 0;   // count of bit of one package
    while (timeout < 2)
    {
        // Read a byte //
        size_t n = ::read(port_, &read_byte, sizeof(read_byte));
        if (n == 0)
        {
            timeout++;
        }
        else if (n == 1)
        {
            // Shift the buffer //
            for (int i = 0; i < 17; i++)
            {
                buff_[i] = buff_[i + 1];
            }
            buff_[17] = read_byte;
            // ROS_INFO_STREAM("READ:" << buff_[17]);
            count++;
        }
    }

    // error exists, flush the buff_[]
    if (count < 17 || !unpack())
    {
        memset(&d_bus_data_, 0, sizeof(d_bus_data_));
        is_update_ = false;
    }
    else
    {
        std::cout << "ch0:" << d_bus_data_.ch0
                  << " ch1:" << d_bus_data_.ch1
                  << " ch2:" << d_bus_data_.ch2
                  << " ch3:" << d_bus_data_.ch3 << std::endl;
        is_update_ = true;
    }
}

bool AsyncUsart::unpack()
{
    d_bus_data_.ch0 = (buff_[0] | buff_[1] << 8) & 0x07FF;
    d_bus_data_.ch0 -= 1024;
    d_bus_data_.ch1 = (buff_[1] >> 3 | buff_[2] << 5) & 0x07FF;
    d_bus_data_.ch1 -= 1024;
    d_bus_data_.ch2 = (buff_[2] >> 6 | buff_[3] << 2 | buff_[4] << 10) & 0x07FF;
    d_bus_data_.ch2 -= 1024;
    d_bus_data_.ch3 = (buff_[4] >> 1 | buff_[5] << 7) & 0x07FF;
    d_bus_data_.ch3 -= 1024;
    // if (!channel_valid(d_bus_data_.ch0) ||
    //     !channel_valid(d_bus_data_.ch1) ||
    //     !channel_valid(d_bus_data_.ch2) ||
    //     !channel_valid(d_bus_data_.ch3))
    // {
    //   return false;
    // }
    /* prevent remote control zero deviation */
    if (d_bus_data_.ch0 <= 10 && d_bus_data_.ch0 >= -10)
        d_bus_data_.ch0 = 0;
    if (d_bus_data_.ch1 <= 10 && d_bus_data_.ch1 >= -10)
        d_bus_data_.ch1 = 0;
    if (d_bus_data_.ch2 <= 10 && d_bus_data_.ch2 >= -10)
        d_bus_data_.ch2 = 0;
    if (d_bus_data_.ch3 <= 10 && d_bus_data_.ch3 >= -10)
        d_bus_data_.ch3 = 0;

    d_bus_data_.s1 = ((buff_[5] >> 4) & 0x000C) >> 2;
    d_bus_data_.s0 = (buff_[5] >> 4) & 0x0003;

    if ((abs(d_bus_data_.ch0) > 660) || (abs(d_bus_data_.ch1) > 660) || (abs(d_bus_data_.ch2) > 660) ||
        (abs(d_bus_data_.ch3) > 660))
    {
        is_success = false;
        return false;
    }
    d_bus_data_.x = buff_[6] | (buff_[7] << 8);
    d_bus_data_.y = buff_[8] | (buff_[9] << 8);
    d_bus_data_.z = buff_[10] | (buff_[11] << 8);
    d_bus_data_.l = buff_[12];
    d_bus_data_.r = buff_[13];
    d_bus_data_.key = buff_[14] | buff_[15] << 8; // key board code
    d_bus_data_.wheel = (buff_[16] | buff_[17] << 8) - 1024;
    is_success = true;

    return true;
}