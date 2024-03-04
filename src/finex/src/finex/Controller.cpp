#include "finex/Controller.hpp"

namespace finex
{

Controller::Controller()
: Node("control_node")
{
    pot_sub_ = create_subscription<std_msgs::msg::UInt16>("/finex/readings/finex_pot", 100, 
    std::bind(&Controller::pot_callback, this, _1));

    gauge_sub_ = create_subscription<std_msgs::msg::Float32>("/finex/readings/finex_gauge", 100, 
    std::bind(&Controller::gauge_callback, this, _1));

    vel_pub_ = create_publisher<std_msgs::msg::Float64>("/finex/velocity", 100);

    declare_parameter("vel", 2.0);
}

double
Controller::update()
{
    vel = this->get_parameter("vel").as_double();

    vel = std::min(vel, 9.0);
    vel = std::max(vel, -9.0);

    printf("Force: %f\n", force_);
    printf("Angle: %d\n", angle_);
    publish_vel();

    return vel;
}

double
Controller::get_vel()
{
    return vel;
}

void
Controller::publish_vel()
{
    vel_pub_->publish(vel);
}

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

} // namespace finex
