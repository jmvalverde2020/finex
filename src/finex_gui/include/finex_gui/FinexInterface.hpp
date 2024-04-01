#ifndef FINEX_INTERFACE_HPP
#define FINEX_INTERFACE_HPP


#include <QMainWindow>
#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QProgressBar>
#include <QDial>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class FinexInterface; }
QT_END_NAMESPACE

class FinexInterface : public QMainWindow
{
    Q_OBJECT

public:
    FinexInterface(QWidget *parent = nullptr);
    ~FinexInterface();

private:

    int set_start();
    int set_stop();

    int record();

    int change_mode(int mode);

    int set_loop();
    int set_sit();
    int set_stand();
    int change_trajectory(int path);

    int set_impedance_level(int level);

    void show_progress();

    bool recording = false;

    Ui::FinexInterface *ui;
    QSlider *impedance_level;
    QDial *control_dial;
    QProgressBar *progress;

    rclcpp::SyncParametersClient::SharedPtr  parameters_client;
    rclcpp::Node::SharedPtr node;
};

#endif // FINEX_INTERFACE_HPP