#include "finex/Controller.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

enum gains {
    KP = 0.144,
    KD = 0.0,
    KI = 0.2866
};

namespace finex
{

Controller::Controller()
: Node("control_node")
{
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    pot_sub_ = this->create_subscription<std_msgs::msg::UInt16>("/finex/readings/finex_pot", sensor_qos, 
    std::bind(&Controller::pot_callback, this, _1));

    gauge_sub_ = this->create_subscription<std_msgs::msg::Float32>("/finex/readings/finex_gauge", sensor_qos, 
    std::bind(&Controller::gauge_callback, this, _1));

    vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/finex/velocity", sensor_qos);
    goal_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/finex/goal_pot", sensor_qos);

    time_out = this->create_wall_timer(
      2ms, std::bind(&Controller::timeOut_callback, this));

    declare_parameter("vel", 1.0);
    declare_parameter("angle", 45);
    declare_parameter("force", 0.0);

    declare_parameter("kp", KP);
    declare_parameter("kd", KD);
    declare_parameter("ki", KI);
}

void
Controller::init(double ts)
{
    KP_ = KP;
    KI_ = KI;
    KD_ = KD;

    Ts = ts;

    cp = 0.0;
    ci = 0.0;
    cd = 0.0;

    vel = 0.0;
    prev_error = 0.0;
}

void
Controller::init(double kp, double ki, double kd, double ts)
{
    KP_ = kp;
    KI_ = ki;
    KD_ = kd;

    Ts = ts;

    cp = 0.0;
    ci = 0.0;
    cd = 0.0;
}

double
Controller::update()
{
    double error;

    switch(control_mode) {
        case 0:
            error = p_update();
            break;

        case 1:
            error = f_update();
            break;

        default:
            return OFFSET;
    }

    vel = apply_PID(error);
    vel = check_limits();

    // vel = this->get_parameter("vel").as_double(); // For debuging purposes
    publish_vel();

    printf("velocity: %f\n", vel);
    vel = vel+OFFSET;
    return vel;
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

void 
Controller::timeOut_callback()
{
    angle_ = -1;
    force_ = 0.0;
}

void
Controller::publish_vel()
{
    auto msg = std_msgs::msg::Float64();
    msg.data = vel;
    vel_pub_->publish(msg);
}

void
Controller::publish_goal(int goal)
{
    auto msg = std_msgs::msg::UInt16();
    msg.data = goal;
    goal_pub_->publish(msg);
}

double
Controller::p_update()
{
    int goal = this->get_parameter("angle").as_int();

    if (angle_ < 0) {
        return 0.0;
    }

    if (!(goal > 0 && goal < 90) || !(angle_ > 0 && angle_ < 90)) {
        return prev_error;
    }

    publish_goal(goal);
    double error = static_cast<double>(goal - angle_);

    return error;
}

double
Controller::f_update()
{
    int goal = this->get_parameter("force").as_double();
    double error = goal - force_;

    return error;
}

double
Controller::apply_PID(double error)
{
    KP_ = this->get_parameter("kp").as_double();
    KD_ = this->get_parameter("kd").as_double();
    KI_ = this->get_parameter("ki").as_double();

    // Proportional controller
    cp = error * KP_;

    printf("Kp = %f, cp = %f\n", KP_, cp);

    // Integrative controller
    ci += error * Ts * KI_;

    // Derivative controller
    cd = ((error - prev_error) / Ts) * KD_;

    vel = cp + ci + cd;

    prev_error = error;

    return vel;
}

double
Controller::check_limits()
{
    vel = std::clamp(vel, MINV, MAXV);
    if (angle_ < 5 && vel < 0.0) {
        vel = 0.0;
    }
    else if (angle_ > 85 && vel > 0.0) {
        vel = 0.0;
    }

    return vel;
}
} // namespace finex
