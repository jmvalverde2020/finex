 
#ifndef FINEX_CONTROLLER_HPP_
#define FINEX_CONTROLLER_HPP_

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace finex
{

class Controller : public rclcpp::Node
{
public:
    Controller();

    double update();

    double get_vel();

private:
    void pot_callback(std_msgs::msg::UInt16 msg);

    void gauge_callback(std_msgs::msg::Float32 msg);

    void publish_vel();

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gauge_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;

    double vel;
    int angle_;
    float force_;
    
};

} // namespace finex

#endif // FINEX_CONTROLLER_HPP_
