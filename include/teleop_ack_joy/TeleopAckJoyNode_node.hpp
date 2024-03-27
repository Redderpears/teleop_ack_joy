#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TeleopAckJoyNode : public rclcpp::Node {
private:
    // Sub
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    // Pub
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub;

public:
    TeleopAckJoyNode(const rclcpp::NodeOptions& options);

    float map_input(float in, float inMin, float inMax, float outMin, float outMax);

    void joy_cb(sensor_msgs::msg::Joy::SharedPtr outputs);
};
