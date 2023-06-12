#ifndef FRANCORGUINODE_H_
#define FRANCORGUINODE_H_

#include <rclcpp/rclcpp.hpp>

#include <QMainWindow>
#include <QTimer>
#include "FrancorWidget.hpp"

#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow, public rclcpp::Node
{
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = 0);
  ~MainWindow() override;

private slots:
  void timer_spin_callback();

  void btn_vid_disable_clicked()
  {
    std::cout << "vid_dis" << std::endl;
    _sub_cam_manip.shutdown();
    _sub_cam_thermo.shutdown();
    _sub_cam_tele_backup.shutdown();
    _sub_cam_tele_main.shutdown();
    _sub_hazmat.shutdown();
    _sub_cam_drive = image_transport::create_subscription(
      this, "/camera/drive/image_raw",
      std::bind(&MainWindow::callbackCameraDrive, this, std::placeholders::_1),
      "compressed", rmw_qos_profile_sensor_data);

    // /camera/fish/ir/image_raw
    // _sub_cam_manip = image_transport::create_subscription(
    //   this, "/camera/fish/ir/image_raw",
    //   std::bind(&MainWindow::callbackCameraManip, this, std::placeholders::_1),
    //   "compressed", rmw_qos_profile_sensor_data);

    _sub_cam_tele_backup = image_transport::create_subscription(
      this, "/camera/fish/ir/image_raw",
      std::bind(&MainWindow::callbackCameraTele_backup, this, std::placeholders::_1),
      "compressed", rmw_qos_profile_sensor_data);

    this->show_black();
  }
  void btn_vid_manip_clicked()
  {
    //sub tele to backup and maip to manip
    _sub_cam_thermo.shutdown();
    _sub_hazmat.shutdown();
    _sub_cam_tele_main.shutdown();

    //tmp hack
    _sub_cam_drive.shutdown();
    _sub_cam_drive = image_transport::create_subscription(
      this, "/camera/drive/image_raw",
      std::bind(&MainWindow::callbackCameraDrive, this, std::placeholders::_1),
      "compressed", rmw_qos_profile_sensor_data);

    // std::cout << "vid_manip" << std::endl;
    // _sub_cam_manip = image_transport::create_subscription(
    //   this, "/camera/front/image_raw",
    //   std::bind(&MainWindow::callbackCameraDrive, this, std::placeholders::_1),
    //   "compressed", rmw_qos_profile_sensor_data);
    _sub_cam_manip = image_transport::create_subscription(
      this, "/camera/manipulator/image_raw",
      std::bind(&MainWindow::callbackCameraManip, this, std::placeholders::_1),
      "compressed", rmw_qos_profile_sensor_data);


    _sub_cam_tele_backup = image_transport::create_subscription(
      this, "/camera/tele/image_raw",
      std::bind(&MainWindow::callbackCameraTele_backup, this, std::placeholders::_1),
      "compressed", rmw_qos_profile_sensor_data);

  }
  void btn_vid_haz_clicked()
  {
    //sub tele to main, haz to manip and thermo to backup
    _sub_cam_tele_backup.shutdown();
    _sub_cam_tele_main.shutdown();
    _sub_cam_manip.shutdown();
    _sub_cam_drive.shutdown();
    _sub_hazmat.shutdown();

    _sub_hazmat = image_transport::create_subscription(
      this, "/find_object_2d/img_hazmats",
      std::bind(&MainWindow::callbackCameraHaz, this, std::placeholders::_1),
      "compressed", rmw_qos_profile_sensor_data);

    _sub_cam_thermo = image_transport::create_subscription(
      this, "/image_raw",
      std::bind(&MainWindow::callbackCameraThermo, this, std::placeholders::_1),
      "compressed", rmw_qos_profile_default);
    std::cout << "vid_haz" << std::endl;


    _sub_cam_tele_backup = image_transport::create_subscription(
      this, "/camera/tele/image_raw",
      std::bind(&MainWindow::callbackCameraTele_backup, this, std::placeholders::_1),
      "compressed", rmw_qos_profile_sensor_data);
  }

  void show_black();

private: 
  void sub_str_a_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // std::cout << "sub_str_callback: " << msg->data << std::endl;
    _francor_widget->setInfoA(msg->data);
  }

  void sub_str_b_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // std::cout << "sub_str_callback: " << msg->data << std::endl;
    _francor_widget->setInfoB(msg->data);
  }

  void sub_str_c_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // std::cout << "sub_str_callback: " << msg->data << std::endl;
    _francor_widget->setInfoC(msg->data);
  }

  void sub_co2_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::cout << "sub_co2_callback: " << msg->data << std::endl;
    _francor_widget->setCo2Value((int)(msg->data));
    // _francor_widget->seti
  }

  void callbackCameraDrive(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void callbackCameraManip(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void callbackCameraHaz(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  
  void callbackCameraTele_main(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void callbackCameraTele_backup(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void callbackTempFrontRight(const std_msgs::msg::Float32::SharedPtr msg);
  void callbackTempFrontLeft(const std_msgs::msg::Float32::SharedPtr msg);
  void callbackTempRearRight(const std_msgs::msg::Float32::SharedPtr msg);
  void callbackTempRearLeft(const std_msgs::msg::Float32::SharedPtr msg);


  void callbackCameraThermo(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  cv::Mat resize_to(cv::Mat img, const int width, const int height)
  {
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(width, height));
    return resized;
  }

  cv::Mat resize_to400(cv::Mat img)
  {
    return resize_to(img, 400, 320);
  }

  cv::Mat resize_to800(cv::Mat input)
  {
    // cv::Mat output;
    // cv::resize(input, output, cv::Size(800, 600));
    // return output;
    return resize_to(input, 800, 600);
  }

private: 

  Ui::MainWindow* ui;

  francor::FrancorWidget* _francor_widget;
  QTimer* _timer_spin;  

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_string_a;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_string_b;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_string_c;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sub_temp_wheel_front_right;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sub_temp_wheel_front_left;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sub_temp_wheel_back_right;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sub_temp_wheel_back_left;
  image_transport::Subscriber _sub_cam_drive;
  image_transport::Subscriber _sub_cam_thermo;
  image_transport::Subscriber _sub_cam_manip;
  image_transport::Subscriber _sub_hazmat;
  image_transport::Subscriber _sub_cam_tele_main;
  image_transport::Subscriber _sub_cam_tele_backup;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sub_co2;

  cv::Mat _img_drive;
};




#endif  //FRANCORGUINODE_H_