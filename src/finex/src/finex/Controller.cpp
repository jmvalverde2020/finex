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

    init_params();
}

void
Controller::init_params()
{
    this->declare_parameter("vel", 1.0);
    this->declare_parameter("angle", 45);

    // For tunning the controller
    this->declare_parameter("kp", 0.1);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("ki", 0.0);

    // For tunning the impedance model
    this->declare_parameter("ks", 0.1);

    // Params for controlling via GUI
    this->declare_parameter("trajectory", 0);
    this->declare_parameter("impedance_level", 0);
    this->declare_parameter("gait_assistance", 1.0);
    this->declare_parameter("progress", 0);

    // Init Start param and callback
    this->declare_parameter("start", 0);
    start_param_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto set_start = [this](const rclcpp::Parameter & p) {
        start = p.as_int();

        if (!start) {
            printf("STOPPED\n");
        }
    };
    start_cb_handle_ = start_param_->add_parameter_callback("start", set_start);

    // Init Record param and callback
    this->declare_parameter("record", false);
    record_param_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto set_record = [this](const rclcpp::Parameter & p) {
        record = p.as_bool();

        if (record) {
            printf("RECORDING\n");
        } else {
            printf("RECORDING ENDED\n");
        }
    };
    record_cb_handle_ = record_param_->add_parameter_callback("record", set_record);

    // Init Mode param and callback
    this->declare_parameter("mode", 1);
    mode_param_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto set_mode = [this](const rclcpp::Parameter & p) {
        control_mode = p.as_int();
        set_gains();
    };
    mode_cb_handle_ = mode_param_->add_parameter_callback("mode", set_mode);
}

int
Controller::init(double ts)
{  
    control_mode = GAIT;

    if (!set_gains()) {
        return 0;
    }

    Ts = ts;

    cp = 0.0;
    ci = 0.0;
    cd = 0.0;

    vel = 0.0;
    prev_error = 0.0;

    return 1;
}

void
Controller::init(double kp, double ki, double kd, double ts)
{
    control_mode = GAIT;

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

    if (!start) {
        // printf("STOP\n");
        return OFFSET;
    }

    // printf("RUNNING\n");

    switch(control_mode) {
        case POSITION:
            p_goal = this->get_parameter("angle").as_int();
            error = p_update(p_goal);
            vel = apply_PID(error);
            break;

        case GAIT:
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
            printf("No control\n");
            return OFFSET;
    }

    // vel = this->get_parameter("vel").as_double(); // For debuging purposes
    
    vel = check_limits();

    // printf("velocity: %f\n", vel);
    publish_vel();

    vel = vel+OFFSET;

    return vel;
}

int
Controller::set_gains()
{
    switch (control_mode) {
        case POSITION:
            KP_ = KP_A;
            KI_ = KI_A;
            KD_ = KD_A;
            break;

        case GAIT:
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
    rclcpp::Parameter("ks", KS_),
    rclcpp::Parameter("progress", 0)
    };

    auto results = this->set_parameters(all_new_parameters);

    // Check to see if it was set.
    for (auto & result : results) {
        if (!result.successful) {
        fprintf(stderr, "Failed to set parameter: %s", result.reason.c_str());
        return 0;
        }
    }

    return 1;
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
        return std::nan("");
    }

    if (goal < P_MIN || goal > P_MAX) {
        printf("Goal: %d, Trajectory: %d\n", goal, t_path);
        RCLCPP_ERROR(this->get_logger(), "Goal out of bounds");
        return std::nan("");
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
        return std::nan("");
    }

    std::clamp(goal, F_MIN, F_MAX);

    if (force_ < F_MIN || force_ > F_MAX) {
        return std::nan("");
    }

    publish_force(goal);

    error = goal - force_;

    return error;
}

double
Controller::apply_PID(double error)
{   
    if (std::isnan(error)) {
        return 0.0;
    }

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

    if (t_path == GAIT) {
        double w = this->get_parameter("gait_assistance").as_double();
        vel = vel * w;
    }
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
Controller::t_clear()
{
    std::vector<rclcpp::Parameter> param {rclcpp::Parameter("trajectory", FREE)};
    auto set_results = this->set_parameters(param);

    // Check to see if it was set.
    for (auto & result : set_results) {
        if (!result.successful) {
            fprintf(stderr, "Failed to set parameter: %s", result.reason.c_str());
            return 0;
        }
    }

    return 1;
}

int
Controller::t_loop()
{
    if (t_state == START) {
        t_state = GO;
        t_goal = P_MIN;
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
            t_state = END;
            t_path = FREE;
            t_clear();
            return -1;
        }
    }

    return t_goal;
}

int
Controller::t_squat()
{
    if (t_state == START) {
        t_state = BACK;
        t_goal = P_MAX;
    }
    else if (t_state == GO) {
        if (t_goal == angle_) {
            t_goal = angle_ + 1;
        }

        if (t_goal == P_MAX) {
            t_state = END;
            t_path = FREE;
            t_clear();
            return -1;
        }
    }
    else if (t_state == BACK) {
        if (t_goal == angle_) {
            t_goal = angle_ - 1;
        }

        if (t_goal == P_MIN) {
            t_state = BACK;
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
            t_clear();
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
            t_clear();
            return -1;
        }
    }

    return t_goal;
}

int
Controller::get_trajectory()
{
    printf("t_path: %d, t_state: %d\n", t_path, t_state);

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
        
        case SQUAT:
            t_goal = t_squat();
            break;

        default:
            t_path = this->get_parameter("trajectory").as_int();
            t_goal = -1;
            t_state = START;
            break;
    }

    

    return t_goal;
}

void
Controller::check_progress()
{
    int value;
    
    if ( t_state == GO )  {
        value = static_cast<int>(angle_ * 1.29);
        if ( t_path == LOOP ) {
            value = value / 2;

            value = std::min(value, 50);

        } else if ( t_path == SQUAT ) {
            value = 50 + (value / 2);

            value = std::min(value, 100);
        }
    } else if ( t_state == BACK ) {
        value = static_cast<int>((P_MAX - angle_) * 1.29);
        if ( t_path == SQUAT ) {
            value = value / 2;

            value = std::min(value, 50);
        } else if ( t_path == LOOP ) {
            value = 50 + (value / 2);

            value = std::min(value, 100);
        }
    }

    std::vector<rclcpp::Parameter> param {rclcpp::Parameter("progress", value)};
    auto set_results = this->set_parameters(param);

    // Check to see if it was set.
    for (auto & result : set_results) {
        if (!result.successful) {
        fprintf(stderr, "Failed to set parameter: %s", result.reason.c_str());
        }
    }
}

double
Controller::impedance()
{
    int goal, level;
    double error, imp;

    level = this->get_parameter("impedance_level").as_int();

    KS_ = (level * KS_MAX) / 5.0;

    // printf("Impedance gain: %f\n", KS_);

    goal = get_trajectory();

    printf("Goal: %d\n", goal);

    if (t_path == FREE) {
        return 0.0;
    }

    error = p_update(goal);

    if (std::isnan(error)) {
        return error;
    }
    check_progress();

    imp = error * KS_;
    // printf("Impedancia: %f\n", imp);

    imp = std::clamp(imp, I_MIN, I_MAX);

    // printf("Impedancia Real: %f\n", imp);

    return imp;

}

} // namespace finex
