 
#ifndef FINEX_CONTROLLER_HPP_
#define FINEX_CONTROLLER_HPP_

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

enum modes {
    POSITION = 0,
    GAIT,
    IMPEDANCE
};

enum trajectories {
    FREE = 0,
    LOOP,
    SIT,
    STAND,
    SQUAT
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

    int init(double ts);
    void init(double kp, double ki, double kd, double ts);

    double update();
private:

    void init_params();
    int set_gains();
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

    int t_clear();

    int t_stand();
    int t_sit();
    int t_loop();
    int t_squat();

    int get_trajectory();
    void check_progress();

    double impedance();

    const std::string get_date_time();
    const std::string ask_bag_name();

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pot_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gauge_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;
    rclcpp::TimerBase::SharedPtr time_out;

    std::shared_ptr<rclcpp::ParameterEventHandler> start_param_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> start_cb_handle_;

    std::shared_ptr<rclcpp::ParameterEventHandler> record_param_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> record_cb_handle_;

    std::shared_ptr<rclcpp::ParameterEventHandler> mode_param_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> mode_cb_handle_;

    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::string bag_name;

    double KP_, KI_, KD_, KS_;
    double cp, ci, cd;
    double prev_error, Ts;

    double vel;

    int t_path = FREE;
    int t_state = START, t_goal = P_MIN;

    int angle_;
    float force_;

    int start = 0;
    bool record = false;
    // Control modes: positon -> 0, gait -> 1, impedance -> 2.
    int control_mode=GAIT;

    /*--------CONSTANTS-------*/

    const double F_THRESHOLD = 0.58;

    // Control constants
    const double V_MIN = -5.0, V_MAX = 5.0;
    const double F_MIN = -14, F_MAX = 14;
    const double I_MIN = -10, I_MAX = 10;
    const double P_MIN = 5, P_MAX = 85;
    const double OFFSET=0.781;

    // Gains for position control
    const double KP_A = 0.144;
    const double KI_A = 0.2866;
    const double KD_A = 0.0;

    // Gains for transparent control
    const double KP_T = 0.15;
    const double KI_T = 0.0;
    const double KD_T = 0.0;
    const double W_MAX = 4.0, W_LEVELS = 4.0;

    // Gains for impredance
    const double KS_I = 0.7, KS_MAX = 5, KS_LEVELS = 5.0;
};

} // namespace finex

#endif // FINEX_CONTROLLER_HPP_
