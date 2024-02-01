#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/UInt16.hpp"

using std::placeholders::_1;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode()
    : Node("control_node")
    {
        pot_sub_ = this->create_subscription<std_msgs::msg::UInt16>("reflex_pot"), 100, 
        std::bind(&ControlNode::pot_callback, this, _1);

        gauge_sub_ = this->create_subscription<std_msgs::msg::UInt16>("reflex_gauge"), 100, 
        std::bind(&ControlNode::gauge_callback, this, _1);
    }

private:
    void pot_callback(const std_msgs::msg::UInt16 & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Recibido del potenciómetro: '%d'", msg.data);
    }

    void gauge_callback(const std_msgs::msg::UInt16 & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Recibido de la galga: '%d'", msg.data);
    }

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pot_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr gauge_sub_;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}