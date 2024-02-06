#include "reflex-control-cpp/control_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<reflex_control::ControlNode>());
    rclcpp::shutdown();
    return 0;
}