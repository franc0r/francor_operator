#ifndef FRANCORGUINODE_H_
#define FRANCORGUINODE_H_

#include <rclcpp/rclcpp.hpp>

#include <QMainWindow>
#include <QTimer>
#include "FrancorWidget.hpp"

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>


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

private: 
  void sub_str_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "sub_str_callback: " << msg->data << std::endl;
  }

  void sub_co2_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    _francor_widget->setCo2Value((int)msg->data);
  }

private: 

  Ui::MainWindow *ui;

  francor::FrancorWidget* _francor_widget;
  QTimer* _timer_spin;  

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_string;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sub_co2;

};




#endif  //FRANCORGUINODE_H_