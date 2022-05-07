#ifndef FRANCORGUINODE_H_
#define FRANCORGUINODE_H_

#include <rclcpp/rclcpp.hpp>

#include <QMainWindow>
#include <QTimer>
#include "FrancorWidget.hpp"

#include <std_msgs/msg/string.hpp>



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

private: 

  Ui::MainWindow *ui;

  francor::FrancorWidget* _francor_widget;
  QTimer* _timer_spin;  

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_string;
};




#endif  //FRANCORGUINODE_H_