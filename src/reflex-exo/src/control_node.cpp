
#include "reflex-exo/control_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<reflex_exo::ControlNode>());
    rclcpp::shutdown();
    return 0;
}