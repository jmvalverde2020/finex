
#ifndef REFLEX_EXO_CONTROLNODE_HPP_
#define REFLEX_EXO_CONTROLNODE_HPP_

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int16.hpp"

using std::placeholders::_1;

namespace reflex_exo
{

class ControlNode : public rclcpp::Node
{
public:
    ControlNode()
    : Node("control_node")
    {
        pot_sub_ = this->create_subscription<std_msgs::msg::UInt16>("/reflex/readings/gauge_raw", 100, 
        std::bind(&ControlNode::pot_callback, this, _1));

        gauge_sub_ = this->create_subscription<std_msgs::msg::Float32>("/reflex/readings/reflex_gauge", 100, 
        std::bind(&ControlNode::gauge_callback, this, _1));

        this->declare_parameter("vel", 2.0);
    }

    int angle;
    float force;

private:
    void pot_callback(std_msgs::msg::UInt16 msg)
    {
        RCLCPP_INFO(this->get_logger(), "Recibido de la galga (raw): '%d'", msg.data);
        angle = msg.data;
    }

    void gauge_callback(std_msgs::msg::Float32 msg)
    {
        RCLCPP_INFO(this->get_logger(), "Recibido de la galga: '%f'", msg.data);
        force = msg.data;
    }

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gauge_sub_;
};

} // namespace reflex_exo

#endif // REFLEX_EXO_CONTROLNODE_HPP_