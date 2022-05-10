#include "FrancorGuiNode.hpp"

#include "ui_francor_gui.h"

#include <cv_bridge/cv_bridge.h>

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
  _sub_co2 = this->create_subscription<std_msgs::msg::Float32>("/co2_level", 10, std::bind(&MainWindow::sub_co2_callback, this, std::placeholders::_1));

  // image_transport::Subscriber _sub_cam_drive;
  // //image_transport::Subscriber _sub_cam_thermo;

  _sub_cam_drive = image_transport::create_subscription(
      this, "/camera//mode",
      std::bind(&MainWindow::callbackCameraDrive, this, std::placeholders::_1),
      hints.getTransport(), rmw_qos_profile_sensor_data, subscription_options);

  _timer_spin->start(100);
}

void MainWindow::callbackCameraDrive(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
  }
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