 
#ifndef FINEX_CONTROLLER_HPP_
#define FINEX_CONTROLLER_HPP_

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

enum modes {
    POSITION = 0,
    TRANSPARENT,
    IMPEDANCE
};

enum trajectories {
    FREE = 0,
    LOOP,
    SIT,
    STAND
};

enum states {
    BACK = -1,
    START,
    GO,
    END
};

namespace finex
{

class Controller : public rclcpp::Node
{
public:
    Controller();

    int init(double ts, int mode);
    void init(double kp, double ki, double kd, double ts, int mode);

    double update();
private:
    void pot_callback(std_msgs::msg::UInt16 msg);
    void gauge_callback(std_msgs::msg::Float32 msg);
    void timeOut_callback();

    void publish_vel();
    void publish_angle(int goal);
    void publish_force(double goal);

    double f_update(double goal);
    double p_update(int goal);

    double apply_PID(double error);

    double check_limits();

    int t_stand();
    int t_sit();
    int t_loop();

    void ask_trajectory();
    int get_trajectory();

    double impedance();

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gauge_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;
    rclcpp::TimerBase::SharedPtr time_out;

    double KP_, KI_, KD_, KS_;
    double cp, ci, cd;
    double prev_error, Ts;

    double vel;

    int t_path = FREE;
    int t_state = START, t_goal = P_MIN;

    int angle_;
    float force_;

    // Control modes: positon -> 0, transparent -> 1.
    int control_mode=POSITION;

    /*--------CONSTANTS-------*/

    const double F_THRESHOLD = 0.58;

    // Control constants
    const double V_MIN = -3.0, V_MAX = 3.0;
    const double F_MIN = -14, F_MAX = 14;
    const double I_MIN = -10, I_MAX = 10;
    const double P_MIN = 5, P_MAX = 85;
    const double OFFSET=0.76;

    // Gains for position control
    const double KP_A = 0.144;
    const double KI_A = 0.2866;
    const double KD_A = 0.0;

    // Gains for transparent control
    const double KP_T = 0.15;
    const double KI_T = 0.0;
    const double KD_T = 0.0;

    // Gains for impredance
    const double KS_I = 0.7, KS_MAX = 3;
};

} // namespace finex

#endif // FINEX_CONTROLLER_HPP_
