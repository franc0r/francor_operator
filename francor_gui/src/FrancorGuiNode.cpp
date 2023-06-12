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
  _sub_string_c = this->create_subscription<std_msgs::msg::String>("/joy2vel/twist_info", 10, std::bind(&MainWindow::sub_str_c_callback, this, std::placeholders::_1));

  _sub_co2 = this->create_subscription<std_msgs::msg::Float32>("/co2_level", rclcpp::QoS(1).best_effort(), std::bind(&MainWindow::sub_co2_callback, this, std::placeholders::_1));

  _sub_temp_wheel_front_right = this->create_subscription<std_msgs::msg::Float32>("/francor_frank_base/drive_front_left/temp", rclcpp::QoS(2).best_effort(), std::bind(&MainWindow::callbackTempFrontRight, this, std::placeholders::_1));
  _sub_temp_wheel_front_left  = this->create_subscription<std_msgs::msg::Float32>("/francor_frank_base/drive_front_right/temp", rclcpp::QoS(2).best_effort(), std::bind(&MainWindow::callbackTempFrontLeft, this, std::placeholders::_1));
  _sub_temp_wheel_back_right  = this->create_subscription<std_msgs::msg::Float32>("/francor_frank_base/drive_rear_left/temp", rclcpp::QoS(2).best_effort(), std::bind(&MainWindow::callbackTempRearRight, this, std::placeholders::_1));
  _sub_temp_wheel_back_left   = this->create_subscription<std_msgs::msg::Float32>("/francor_frank_base/drive_rear_right/temp", rclcpp::QoS(2).best_effort(), std::bind(&MainWindow::callbackTempRearLeft, this, std::placeholders::_1));


  // image_transport::Subscriber _sub_cam_drive;
  // //image_transport::Subscriber _sub_cam_thermo;









  _timer_spin->start(5);
  // cv::namedWindow("drive");
}

void MainWindow::show_black()
{
  cv::Mat black(cv::Size(400, 320), CV_8UC3, cv::Scalar(0, 0, 0));

  QImage imgIn= QImage((uchar*) black.data, black.cols, black.rows, black.step, QImage::Format_RGB888);
  // ui->label_video_drive->setPixmap(QPixmap::fromImage(imgIn));
  ui->label_video_backup->setPixmap(QPixmap::fromImage(imgIn));
  ui->label_video_manip->setPixmap(QPixmap::fromImage(imgIn));
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

  cv::flip(scaled, scaled, -1);

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

    cv::Mat scaled = this->resize_to800(img);

     QImage imgIn= QImage((uchar*) scaled.data, scaled.cols, scaled.rows, scaled.step, QImage::Format_RGB888);
    //  ui->label_video_backup->setPixmap(QPixmap::fromImage(imgIn));
     ui->label_video_drive->setPixmap(QPixmap::fromImage(imgIn));
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
     ui->label_video_manip->setPixmap(QPixmap::fromImage(imgIn));
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


void MainWindow::callbackTempFrontRight(const std_msgs::msg::Float32::SharedPtr msg)
{
  _francor_widget->setTempFR(msg->data);
}
void MainWindow::callbackTempFrontLeft(const std_msgs::msg::Float32::SharedPtr msg)
{
  _francor_widget->setTempFL(msg->data);
}
void MainWindow::callbackTempRearRight(const std_msgs::msg::Float32::SharedPtr msg)
{
  _francor_widget->setTempBR(msg->data);
}
void MainWindow::callbackTempRearLeft(const std_msgs::msg::Float32::SharedPtr msg)
{
  _francor_widget->setTempBL(msg->data);
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