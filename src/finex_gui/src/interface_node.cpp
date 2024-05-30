#include "finex_gui/FinexInterface.hpp"

#include <QApplication>

static void siginthandler(int /*param*/)
{
    QApplication::quit();
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("gui_timer_node");
    FinexInterface w(node);
    w.show();

    app.processEvents();
    while (rclcpp::ok())
    {
      app.processEvents();
      rclcpp::spin_some(node);
    }

    signal(SIGINT, siginthandler);

    rclcpp::shutdown();
    return 0;
}