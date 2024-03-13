 
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

    void init(double ts);
    void init(double kp, double ki, double kd, double ts);

    double update();

    double get_vel();

private:
    void pot_callback(std_msgs::msg::UInt16 msg);

    void gauge_callback(std_msgs::msg::Float32 msg);

    void timeOut_callback();

    void publish_vel();

    void publish_goal(int goal);

    double f_update();

    double p_update();

    double apply_PID(double error);

    double check_limits();

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gauge_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr time_out;

    double cp, cd, ci;
    double prev_error, Ts;

    double vel, MINV = -3.0, MAXV = 3.0;
    double OFFSET=0.76;

    int angle_;
    float force_;
    int control_mode=0;
    
};

} // namespace finex

#endif // FINEX_CONTROLLER_HPP_
