
#include "reflex-exo/control_node.hpp"
#include "reflex-exo/SPI.hpp"

SPI spi;
reflex_exo::ControlNode::SharedPtr node;

void exit_handler(int s) {
    spi.end();
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
    int count = 0;
    std::chrono::high_resolution_clock::time_point stop;
    rclcpp::init(argc, argv);

    node = std::make_shared<reflex_exo::ControlNode>();

    if (!spi.init(node)) {
        RCLCPP_ERROR(node->get_logger(), "SPI initialization failed");
        exit(EXIT_FAILURE);
    }

    auto start = std::chrono::high_resolution_clock::now();

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        vel = get_vel();
        // printf("vel %lf\n", vel);
        spi.sendData(vel);
        count++;
        stop = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<microseconds>(stop - start) > 99999){
            printf("hz = %d\n", count);
        }
    }
    
    spi.end();
    rclcpp::shutdown();
    return 0;
}