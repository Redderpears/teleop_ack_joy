#include "teleop_ack_joy/TeleopAckJoyNode_node.hpp"

// For _1
using namespace std::placeholders;

TeleopAckJoyNode::TeleopAckJoyNode(const rclcpp::NodeOptions& options) : Node("TeleopAckjoyNode", options) {
    joy_sub =
        this->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, std::bind(&TeleopAckJoyNode::joy_cb, this, _1));

    ackermann_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/ack_vel", 1);
}

void TeleopAckJoyNode::joy_cb(sensor_msgs::msg::Joy::SharedPtr outputs) {
    ackermann_msgs::msg::AckermannDrive command;

    float left_stick_lr_val = outputs->axes.at(0);
    float right_trigger_val = -outputs->axes.at(5);

    command.steering_angle = map_input(left_stick_lr_val, -1, 1, -0.2733, 0.2733);
    command.speed = map_input(right_trigger_val, -1, 1, 0, 5);

    ackermann_pub->publish(command);
}

float TeleopAckJoyNode::map_input(float in, float inMin, float inMax, float outMin, float outMax) {
    float slope = (outMax - outMin) / (inMax - inMin);
    float b = outMax - slope * inMax;

    return slope * in + b;
}
