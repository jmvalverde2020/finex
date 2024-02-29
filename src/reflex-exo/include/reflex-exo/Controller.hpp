
#ifndef REFLEX_EXO_CONTROLLER_HPP_
#define REFLEX_EXO_CONTROLLER_HPP_

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int16.hpp"

using std::placeholders::_1;

namespace reflex_exo
{

class Controller : public rclcpp::Node
{
public:
    Controller();

    void update_state();

    double get_state();



private:
    void pot_callback(std_msgs::msg::UInt16 msg);

    void gauge_callback(std_msgs::msg::Float32 msg);

    void publish_vel();

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gauge_sub_;

    double vel;
    int angle_;
    float force_;
    
};

} // namespace reflex_exo

#endif // REFLEX_EXO_CONTROLLER_HPP_
