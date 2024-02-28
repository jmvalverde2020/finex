
#include "reflex-exo/control_node.hpp"
#include "reflex-exo/SPI.hpp"

SPI spi;
reflex_exo::ControlNode::SharedPtr node;

void exit_handler(int s) {
    // spi.end();
    rclcpp::shutdown();
    fprintf(stderr, "Signal %d\n", s);
    exit(0);
}


double get_vel(){
    double vel = node->get_parameter("vel").as_double();

    vel = std::min(vel, 9.0);
    vel = std::max(vel, -9.0);

    return vel;
}

int main(int argc, char * argv[])
{   
    signal(SIGINT, exit_handler);
    double vel;
    rclcpp::init(argc, argv);

    node = std::make_shared<reflex_exo::ControlNode>();

    if (!spi.init(node)) {
        RCLCPP_ERROR(node->get_logger(), "SPI initialization failed");
        exit(EXIT_FAILURE);
    }

    while (rclcpp::ok()) {
        vel = get_vel();
        printf("vel %lf\n", vel);
        spi.sendData(vel);
        rclcpp::spin_some(node);
    }
    
    spi.end();
    rclcpp::shutdown();
    return 0;
}