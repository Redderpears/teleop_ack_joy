#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "teleop_ack_joy/TeleopAckJoyNode_node.hpp"

TEST(TODO_NODE_NAME, Test1) {}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}