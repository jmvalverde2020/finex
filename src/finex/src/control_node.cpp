#include <iostream>
#include "finex/Controller.hpp"
#include "finex/SPI.hpp"

SPI spi;

using namespace std::chrono_literals;

void exit_handler(int s) {
    spi.end();
    rclcpp::shutdown();
    fprintf(stderr, "Signal %d\n", s);
    exit(0);
}

int main(int argc, char * argv[])
{   
    signal(SIGINT, exit_handler);

    int FRQ = 150;
    double Ts = 1.0/FRQ;
    double vel = 0.0;
    // int count = 0;
    std::chrono::high_resolution_clock::time_point stop;

    rclcpp::WallRate loop_rate(FRQ); // 400Hz
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<finex::Controller>();

    if (!spi.init(controller)) {
        RCLCPP_ERROR(controller->get_logger(), "SPI initialization failed");
        exit(EXIT_FAILURE);
    }
    
    if (!controller->init(Ts)){
        RCLCPP_ERROR(controller->get_logger(), "Controller initialization failed");
        exit(EXIT_FAILURE);
    }

    RCLCPP_INFO(controller->get_logger(), "System initiated");
    // auto start = std::chrono::high_resolution_clock::now();
    while (rclcpp::ok()) {
        rclcpp::spin_some(controller);

        vel = controller->update();
        spi.sendData(vel);

        // For debugging frequency
        // count++;
        // stop = std::chrono::high_resolution_clock::now();
        // if (std::chrono::duration_cast<std::chrono::microseconds>(stop - start) > 999ms){
        //     printf("hz = %d\n", count);
        //     start = std::chrono::high_resolution_clock::now();
        //     count = 0;
        // }

        loop_rate.sleep();
    }
    
    spi.end();
    rclcpp::shutdown();
    return 0;
}
