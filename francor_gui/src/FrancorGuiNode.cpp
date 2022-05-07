#include "FrancorGuiNode.hpp"

#include "ui_francor_gui.h"


#include <QtWidgets>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    rclcpp::Node("francor_gui_node"),
    ui(new Ui::MainWindow)
{
	ui->setupUi(this);
  _francor_widget = new francor::FrancorWidget();
  ui->widget_francor->setLayout(_francor_widget->getLayout());

  _timer_spin = new QTimer(this);
  connect(_timer_spin, SIGNAL(timeout()), this, SLOT(timer_spin_callback()));

  _sub_string = this->create_subscription<std_msgs::msg::String>("/joy2vel/mode", 10, std::bind(&MainWindow::sub_str_callback, this, std::placeholders::_1));

  _timer_spin->start(100);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::timer_spin_callback()
{
  std::cout << "timer" << std::endl;
  rclcpp::spin_some(this->get_node_base_interface());
}