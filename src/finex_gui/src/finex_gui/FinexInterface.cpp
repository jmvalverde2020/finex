#include "finex_gui/FinexInterface.hpp"
#include "../../resources/ui_FinexInterface.h"

using namespace std::chrono_literals;

FinexInterface::FinexInterface(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::FinexInterface)
{ 
  ui->setupUi(this);

  connect(ui->Start_B, &QPushButton::pressed, this, &FinexInterface::set_start);
  connect(ui->Stop_B, &QPushButton::pressed, this, &FinexInterface::set_stop);
  connect(ui->Record_B, &QPushButton::pressed, this, &FinexInterface::record);
  connect(ui->Loop_TB, &QPushButton::pressed, this, &FinexInterface::set_loop);
  connect(ui->Sit_TB, &QPushButton::pressed, this, &FinexInterface::set_sit);
  connect(ui->Stand_TB, &QPushButton::pressed, this, &FinexInterface::set_stand);

  connect(ui->Impedance_slider, &QSlider::valueChanged, this, &FinexInterface::set_impedance_level);
  connect(ui->Control_dial, &QDial::valueChanged, this, &FinexInterface::change_mode);

  timer_node = node;
  auto param_node = rclcpp::Node::make_shared("finex_gui_param_node");
  parameters_client = std::make_shared<rclcpp::SyncParametersClient>(param_node, "control_node");

  progress_timer = timer_node->create_wall_timer(
      50ms, std::bind(&FinexInterface::show_progress, this));
}

FinexInterface::~FinexInterface()
{
  delete ui;
}

int
FinexInterface::update_param(std::vector<rclcpp::Parameter> param)
{
  auto set_results = parameters_client->set_parameters(param);

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
FinexInterface::set_start()
{ 
  std::vector<rclcpp::Parameter> param {rclcpp::Parameter("start", 1)};
  
  if (!update_param(param)) {
    return 0;
  }

  return 1;
}

int
FinexInterface::set_stop()
{
  std::vector<rclcpp::Parameter> param {rclcpp::Parameter("start", 0)};

  if (!update_param(param)) {
    return 0;
  }

  return 1;
}

int
FinexInterface::record()
{
  std::vector<rclcpp::Parameter> param {rclcpp::Parameter("record", recording)};

  if (!update_param(param)) {
    return 0;
  }

  recording = !recording;

  return 1;
}

int
FinexInterface::change_mode(int mode)
{
  std::vector<rclcpp::Parameter> param {rclcpp::Parameter("mode", mode)};

  if (!update_param(param)) {
    return 0;
  }

  return 1;
}

int
FinexInterface::set_loop()
{ 
  
  if (!change_trajectory(1)) {
    return 0;
  }


  return 1;
}

int
FinexInterface::set_sit()
{ 
  
  if (!change_trajectory(2)) {
    return 0;
  }


  return 1;
}

int
FinexInterface::set_stand()
{ 
  
  if (!change_trajectory(3)) {
    return 0;
  }


  return 1;
}

int
FinexInterface::change_trajectory(int path)
{
  std::vector<rclcpp::Parameter> param {rclcpp::Parameter("trajectory", path)};
  
  if (!update_param(param)) {
    return 0;
  }

  return 1;
}

int
FinexInterface::set_impedance_level(int level)
{
  std::vector<rclcpp::Parameter> param {rclcpp::Parameter("impedance_level", level)};
  
  if (!update_param(param)) {
    return 0;
  }

  return 1;
}

void
FinexInterface::show_progress()
{ 
  int value;
  value = parameters_client->get_parameters({"progress"})[0].as_int();

  ui->Progress_Bar->setValue(value);
}