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
  _francor_widget = new francor::FrancorWidget(this);
  ui->widget_francor->setLayout(_francor_widget->getLayout());

  _timer_spin = new QTimer(this);
  connect(_timer_spin, SIGNAL(timeout()), this, SLOT(timer_spin_callback()));

  _sub_string_a = this->create_subscription<std_msgs::msg::String>("/joy2vel/mode", 10, std::bind(&MainWindow::sub_str_a_callback, this, std::placeholders::_1));
  _sub_string_b = this->create_subscription<std_msgs::msg::String>("/manipulator/status", 10, std::bind(&MainWindow::sub_str_b_callback, this, std::placeholders::_1));
  // _sub_string_c = this->create_subscription<std_msgs::msg::String>("/joy2vel/mode", 10, std::bind(&MainWindow::sub_str_callback, this, std::placeholders::_1));

  _sub_co2 = this->create_subscription<std_msgs::msg::Float32>("/co2_level", 10, std::bind(&MainWindow::sub_co2_callback, this, std::placeholders::_1));

  // image_transport::Subscriber _sub_cam_drive;
  // //image_transport::Subscriber _sub_cam_thermo;





  _timer_spin->start(5);
  // cv::namedWindow("drive");
}

void MainWindow::callbackCameraDrive(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat img = cv_ptr->image;

    cv::Mat scaled = this->resize_to800(img);

    QImage imgIn= QImage((uchar*) scaled.data, scaled.cols, scaled.rows, scaled.step, QImage::Format_RGB888);
    ui->label_video_drive->setPixmap(QPixmap::fromImage(imgIn));
    // ui->label_video_drive->bac


    // cv::imshow("drive", img);
    // cv::waitKey(5);
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

void MainWindow::callbackCameraManip(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
  // First let cv_bridge do its magic
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
  cv::Mat img = cv_ptr->image;

  cv::Mat scaled = this->resize_to400(img);

  QImage imgIn= QImage((uchar*) scaled.data, scaled.cols, scaled.rows, scaled.step, QImage::Format_RGB888);
  ui->label_video_manip->setPixmap(QPixmap::fromImage(imgIn));

  // cv::imshow("manip", img);
  // cv::waitKey(5);
  }
  catch (cv_bridge::Exception& e)
  {
  }
      // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
}


void MainWindow::callbackCameraHaz(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try
    {
      // First let cv_bridge do its magic
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat img = cv_ptr->image;

    cv::Mat scaled = this->resize_to400(img);

     QImage imgIn= QImage((uchar*) scaled.data, scaled.cols, scaled.rows, scaled.step, QImage::Format_RGB888);
     ui->label_video_backup->setPixmap(QPixmap::fromImage(imgIn));
    // cv::imshow("haz", img);
    // cv::waitKey(5);
    }
    catch (cv_bridge::Exception& e)
    {
      // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }


  void MainWindow::callbackCameraThermo(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try
    {
      // First let cv_bridge do its magic
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat img = cv_ptr->image;


    cv::Mat scaled = this->resize_to(img, 220, 400);

     QImage imgIn= QImage((uchar*) scaled.data, scaled.cols, scaled.rows, scaled.step, QImage::Format_RGB888);
     ui->label_video_backup->setPixmap(QPixmap::fromImage(imgIn));
    // cv::imshow("haz", img);
    // cv::imshow("thermo", img);
    // cv::waitKey(5);
    }
    catch (cv_bridge::Exception& e)
    {
      // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }


void MainWindow::callbackCameraTele_main(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    try
    {
      // First let cv_bridge do its magic
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat img = cv_ptr->image;


    cv::Mat scaled = this->resize_to800(img);

     QImage imgIn= QImage((uchar*) scaled.data, scaled.cols, scaled.rows, scaled.step, QImage::Format_RGB888);
     ui->label_video_drive->setPixmap(QPixmap::fromImage(imgIn));
    // cv::imshow("haz", img);
    // cv::imshow("thermo", img);
    // cv::waitKey(5);
    }
    catch (cv_bridge::Exception& e)
    {
      // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void MainWindow::callbackCameraTele_backup(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    try
    {
      // First let cv_bridge do its magic
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat img = cv_ptr->image;


    cv::Mat scaled = this->resize_to400(img);

     QImage imgIn= QImage((uchar*) scaled.data, scaled.cols, scaled.rows, scaled.step, QImage::Format_RGB888);
     ui->label_video_backup->setPixmap(QPixmap::fromImage(imgIn));
    // cv::imshow("haz", img);
    // cv::imshow("thermo", img);
    // cv::waitKey(5);
    }
    catch (cv_bridge::Exception& e)
    {
      // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::timer_spin_callback()
{
  // std::cout << "timer" << std::endl;
  rclcpp::spin_some(this->get_node_base_interface());
}