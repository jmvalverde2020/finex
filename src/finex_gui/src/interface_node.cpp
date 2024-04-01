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

    FinexInterface w;
    w.show();

    app.processEvents();
    while (rclcpp::ok())
    {
      app.processEvents();
    }

    signal(SIGINT, siginthandler);

    rclcpp::shutdown();
    return 0;
}