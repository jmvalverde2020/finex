#include "finex/Controller.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

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
    angle_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/finex/angle_goal", sensor_qos);
    force_pub_ = this->create_publisher<std_msgs::msg::Float64>("/finex/force_goal", sensor_qos);

    time_out = this->create_wall_timer(
      2ms, std::bind(&Controller::timeOut_callback, this));

    this->declare_parameter("vel", 1.0);
    this->declare_parameter("angle", 45);

    // For tunning the controller
    this->declare_parameter("kp", 0.1);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("ki", 0.0);

    // For tunning the impedance model
    this->declare_parameter("ks", 0.1);
}

int
Controller::init(double ts, int mode)
{  
    control_mode = mode;

    switch (control_mode) {
        case POSITION:
            KP_ = KP_A;
            KI_ = KI_A;
            KD_ = KD_A;
            break;

        case TRANSPARENT:
            KP_ = KP_T;
            KI_ = KI_T;
            KD_ = KD_T;
            break;
        
        case IMPEDANCE:
            KP_ = KP_T;
            KI_ = KI_T;
            KD_ = KD_T;
            KS_ = KS_I;
            break;

        default:
            return 0;
    }
    
    std::vector<rclcpp::Parameter> all_new_parameters{
    rclcpp::Parameter("kp", KP_), 
    rclcpp::Parameter("ki", KI_),
    rclcpp::Parameter("kd", KD_),
    rclcpp::Parameter("ks", KS_)
    };

    this->set_parameters(all_new_parameters);

    Ts = ts;

    cp = 0.0;
    ci = 0.0;
    cd = 0.0;

    vel = 0.0;
    prev_error = 0.0;

    return 1;
}

void
Controller::init(double kp, double ki, double kd, double ts, int mode)
{
    control_mode = mode;

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
    double error, f_goal, p_goal;

    switch(control_mode) {
        case POSITION:
            p_goal = this->get_parameter("angle").as_int();
            error = p_update(p_goal);
            vel = apply_PID(error);
            break;

        case TRANSPARENT:
            f_goal = 0.0;
            error = f_update(f_goal);
            vel = apply_PID(error);
            break;

        case IMPEDANCE:
            f_goal = impedance();
            error = f_update(f_goal);
            vel = apply_PID(error);
            break;

        default:
            printf("STOP\n");
            return OFFSET;
    }

    // vel = this->get_parameter("vel").as_double(); // For debuging purposes
    
    vel = check_limits();

    printf("velocity: %f\n", vel);
    publish_vel();

    vel = vel+OFFSET;

    return vel;
}

void 
Controller::pot_callback(std_msgs::msg::UInt16 msg)
{
    // RCLCPP_INFO(this->get_logger(), "Recibido del potenciometro: '%d'", msg.data);
    
    if (msg.data < 100) {
        angle_ = msg.data;
    } else {
        angle_ = -1;
    }
}

void 
Controller::gauge_callback(std_msgs::msg::Float32 msg)
{
    //RCLCPP_INFO(this->get_logger(), "Recibido de la galga: '%f'", msg.data);
    force_ = msg.data;

    if (abs(force_) < F_THRESHOLD) {
        force_ = 0.0;
    }
}

void 
Controller::timeOut_callback()
{
    angle_ = -1;
    force_ = std::nan("");
}

void
Controller::publish_vel()
{
    auto msg = std_msgs::msg::Float64();
    msg.data = vel;
    vel_pub_->publish(msg);
}

void
Controller::publish_angle(int goal)
{
    auto msg = std_msgs::msg::UInt16();
    msg.data = goal;
    angle_pub_->publish(msg);
}

void
Controller::publish_force(double goal)
{
    auto msg = std_msgs::msg::Float64();
    msg.data = goal;
    force_pub_->publish(msg);
}

double
Controller::p_update(int goal)
{

    if (angle_ < 0 || angle_ > 95) {
        RCLCPP_ERROR(this->get_logger(), "No angle data");
        return 0.0;
    }

    if (goal < P_MIN || goal > P_MAX) {
        printf("Goal: %d, Trajectory: %d\n", goal, t_path);
        RCLCPP_ERROR(this->get_logger(), "Goal out of bounds");
        return 0.0;
    }

    publish_angle(goal);
    double error = static_cast<double>(goal - angle_);

    return error;
}

double
Controller::f_update(double goal)
{   
    double error;

    if (std::isnan(force_)) {
        RCLCPP_ERROR(this->get_logger(), "No force data");
        return 0.0;
    }

    std::clamp(goal, F_MIN, F_MAX);

    if (force_ < F_MIN || force_ > F_MAX) {
        return 0.0;
    }

    publish_force(goal);

    error = goal - force_;

    return error;
}

double
Controller::apply_PID(double error)
{

    // For tunning the controller
    KP_ = this->get_parameter("kp").as_double();
    KD_ = this->get_parameter("kd").as_double();
    KI_ = this->get_parameter("ki").as_double();

    // Proportional controller
    cp = error * KP_;

    // Integrative controller
    ci += error * Ts * KI_;

    // Anti-Windup
    ci = std::clamp(ci, V_MIN, V_MAX);

    // Derivative controller
    cd = ((error - prev_error) / Ts) * KD_;
    vel = cp + ci + cd;

    prev_error = error;

    return vel;
}

double
Controller::check_limits()
{
    vel = std::clamp(vel, V_MIN, V_MAX);
    if (angle_ < P_MIN && vel < 0.0) {
        vel = 0.0;
    }
    else if (angle_ > P_MAX && vel > 0.0) {
        vel = 0.0;
    }

    return vel;
}

int
Controller::t_loop()
{
    if (t_state == START) {
        t_state = GO;
    }
    else if (t_state == GO) {
        if (t_goal == angle_) {
            t_goal = angle_ + 1;
        }

        if (t_goal == P_MAX) {
            t_state = BACK;
            return t_goal;
        }
    }
    else if (t_state == BACK) {
        if (t_goal == angle_) {
            t_goal = angle_ - 1;
        }

        if (t_goal == P_MIN) {
            t_state = GO;
            return t_goal;
        }
    }

    return t_goal;
}

int
Controller::t_sit()
{
    if (t_state == START) {
        if (t_goal == angle_) {
            t_state = BACK;
        }
        t_goal = P_MAX;
    }
    else if (t_state == BACK) {
        if (t_goal == angle_) {
            t_goal = angle_ - 1;
        }

        if (t_goal == P_MIN) {
            t_state = END;
            t_path = FREE;
            return -1;
        }
    }

    return t_goal;
}

int
Controller::t_stand()
{
    if (t_state == START) {
        if (t_goal == angle_) {
            t_state = GO;
        }
        t_goal = P_MIN;
    }
    else if (t_state == GO) {
        if (t_goal == angle_) {
            t_goal = angle_ + 1;
        }

        if (t_goal == P_MAX) {
            t_state = END;
            t_path = FREE;
            return -1;
        }
    }

    return t_goal;
}

void
Controller::ask_trajectory()
{
    int t_aux;

    std::cout << "Please enter the desired trajectory (1 -> loop | 2 -> sit | 3 -> stand)\n";
    std::cin >> t_aux;

    switch (t_aux) {
        case LOOP:
            t_path = LOOP;
            break;
        
        case SIT:
            t_path = SIT;
            break;
        
        case STAND:
            t_path = STAND;
            break;
        
        default:
            std::cout << "Invalid trajectory, please try again\n";
            t_path = FREE;
            break;
    }

    t_state = START;
}

int
Controller::get_trajectory()
{
    switch (t_path) {

        case LOOP:
            t_goal = t_loop();
            break;
        
        case SIT:
            t_goal = t_sit();
            break;
        
        case STAND:
            t_goal = t_stand();
            break;
        
        default:
            ask_trajectory();
            t_goal = -1;
            break;
    }

    return t_goal;
}

double
Controller::impedance()
{
    int goal; // = this->get_parameter("angle").as_int();
    double error, imp;
    KS_ = this->get_parameter("ks").as_double();

    goal = get_trajectory();

    if (t_path == FREE) {
        return 0.0;
    }

    error = p_update(goal);
    imp = error * KS_;
    printf("Impedancia: %f\n", imp);

    imp = std::clamp(imp, I_MIN, I_MAX);

    printf("Impedancia Real: %f\n", imp);

    return imp;

}

} // namespace finex
