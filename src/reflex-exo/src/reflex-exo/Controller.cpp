#include "reflex-exo/Controller.hpp"

namespace reflex_exo
{

Controller::Controller()
: Node("control_node")
{
    pot_sub_ = create_subscription<std_msgs::msg::UInt16>("/reflex/readings/reflex_pot", 100, 
    std::bind(&ControlNode::pot_callback, this, _1));

    gauge_sub_ = create_subscription<std_msgs::msg::Float32>("/reflex/readings/reflex_gauge", 100, 
    std::bind(&ControlNode::gauge_callback, this, _1));

    declare_parameter("vel", 2.0);
}

int angle_;
float force_;

    

void 
Controller::pot_callback(std_msgs::msg::UInt16 msg)
{
    //RCLCPP_INFO(this->get_logger(), "Recibido del potenciometro: '%d'", msg.data);
    angle_ = msg.data;
}

void 
Controller::gauge_callback(std_msgs::msg::Float32 msg)
{
    //RCLCPP_INFO(this->get_logger(), "Recibido de la galga: '%f'", msg.data);
    force_ = msg.data;
}

} // namespace reflex_exo
