
#ifndef REFLEX_CONTROL_CPP_CONTROLNODE_HPP_
#define REFLEX_CONTROL_CPP_CONTROLNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

namespace reflex_control
{

class ControlNode : public rclcpp::Node
{
public:
    ControlNode()
    : Node("control_node")
    {
        pot_sub_ = this->create_subscription<std_msgs::msg::Float32>("/reflex/readings/reflex_pot", 100, 
        std::bind(&ControlNode::pot_callback, this, _1));

        gauge_sub_ = this->create_subscription<std_msgs::msg::Float32>("/reflex/readings/reflex_gauge", 100, 
        std::bind(&ControlNode::gauge_callback, this, _1));
    }

private:
    void pot_callback(const std_msgs::msg::Float32 & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Recibido del potenciómetro: '%f'", msg.data);
    }

    void gauge_callback(const std_msgs::msg::Float32 & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Recibido de la galga: '%f'", msg.data);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gauge_sub_;
};

} // namespace reflex_control

#endif // REFLEX_CONTROL_CPP_CONTROLNODE_HPP_