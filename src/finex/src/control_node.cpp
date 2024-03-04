
#include "finex/Controller.hpp"
#include "finex/SPI.hpp"

using namespace std::chrono_literals;

SPI spi;
finex::Controller::SharedPtr node;

void exit_handler(int s) {
    spi.end();
    rclcpp::shutdown();
    fprintf(stderr, "Signal %d\n", s);
    exit(0);
}

int main(int argc, char * argv[])
{   
    signal(SIGINT, exit_handler);

    double vel;

    int count = 0;
    std::chrono::high_resolution_clock::time_point stop;

    rclcpp::WallRate loop_rate(2500us); // 400Hz
    rclcpp::init(argc, argv);
    node = std::make_shared<finex::Controller>();

    if (!spi.init(node)) {
        RCLCPP_ERROR(node->get_logger(), "SPI initialization failed");
        exit(EXIT_FAILURE);
    }

    auto start = std::chrono::high_resolution_clock::now();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        vel = node->update();
        spi.sendData(vel);

        count++;
        stop = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::microseconds>(stop - start) > 999ms){
            printf("hz = %d\n", count);
            start = std::chrono::high_resolution_clock::now();
            count = 0;
        }

        loop_rate.sleep();
    }
    
    spi.end();
    rclcpp::shutdown();
    return 0;
}
