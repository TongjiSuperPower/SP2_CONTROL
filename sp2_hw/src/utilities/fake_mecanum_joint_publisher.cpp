/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Lithesh
 * All rights reserved.
 */

#include "sp2_hw/utilities/fake_mecanum_joint_publisher.hpp"

namespace utilities
{
    const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
    void fake_joint_publisher::init()
    {
        joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SystemDefaultsQoS());
        realtime_joint_state_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
                joint_state_publisher_);
        auto &joint_state_msg = realtime_joint_state_publisher_->msg_;
        size_t num_joints = name_prefix.size() * name_suffix.size();
        joint_state_msg.name.resize(num_joints);
        joint_state_msg.position.resize(num_joints, 0);
        joint_state_msg.velocity.resize(num_joints, 0);
        joint_state_msg.effort.resize(num_joints, 0);
        for (size_t i = 0; i < name_prefix.size(); ++i)
        {
            for (size_t j = 0; j < name_suffix.size(); ++j)
            {
                joint_state_msg.name[name_suffix.size() * i + j] =
                    name_prefix[i] + "_roller_" + name_suffix[j] + "_joint";
            }
        }
    }

    void fake_joint_publisher::update()
    {
        if (realtime_joint_state_publisher_ && realtime_joint_state_publisher_->trylock())
        {
            auto &joint_state_msg = realtime_joint_state_publisher_->msg_;
            joint_state_msg.header.stamp = rclcpp::Clock().now();
            // 伪造麦轮棍子的数据
            for (size_t i = 0; i < name_prefix.size(); ++i)
            {
                size_t num_suffix = name_suffix.size();
                for (size_t j = 0; j < name_suffix.size(); ++j)
                {
                    size_t index = num_suffix * i + j;
                    joint_state_msg.position[index] = 0;
                    joint_state_msg.velocity[index] = 0;
                    joint_state_msg.effort[index] = 0;
                }
            }
            realtime_joint_state_publisher_->unlockAndPublish();
        }
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fake_mecanum_joint_publisher");
    // 伪发布器没有必要频率很高
    rclcpp::WallRate loop_rate(1.0);
    utilities::fake_joint_publisher fake_joint_publisher(node);
    fake_joint_publisher.init();
    while (rclcpp::ok())
    {
        fake_joint_publisher.update();
        loop_rate.sleep();
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
