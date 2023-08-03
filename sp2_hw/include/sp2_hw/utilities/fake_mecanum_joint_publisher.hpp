/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#ifndef FAKE_MECANUM_JOINT_PUBLISHER
#define FAKE_MECANUM_JOINT_PUBLISHER
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>

namespace utilities
{
    class fake_joint_publisher
    {
    public:
        fake_joint_publisher() = default;
        fake_joint_publisher(std::shared_ptr<rclcpp::Node> node) : node_(node){};
        void init();
        void update();

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
            realtime_joint_state_publisher_;
        std::vector<std::string> name_prefix{"left_front", "left_back", "right_front", "right_back"};
        std::vector<std::string> name_suffix{"0", "1", "2", "3", "4", "5", "6", "7",
                                             "8", "9", "10", "11", "12", "13", "14", "15"};
    };
}
#endif
