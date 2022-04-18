#ifndef my_namespace__my_plugin_H
#define my_namespace__my_plugin_H
 
// #include <rqt_gui_cpp/plugin.h>
// #include <ui_my_plugin.h>
#include <QWidget>

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#endif

class QProgressBar;
class QLineEdit;

namespace my_namespace
{ 

class MyPlugin : public QObject
{ 
  Q_OBJECT

public: 
  MyPlugin(void);
  virtual ~MyPlugin(void) = default;

public slots:
  void stop(void);
  void stopSlam(void);
  void process(void);

private: 
  void callbackCo2SensorFast(const std_msgs::Int32& msg);
  void callbackCo2SensorSlow(const std_msgs::Int32& msg);
  void callbackInfoTextA(const std_msgs::String& msg);
  void callbackInfoTextB(const std_msgs::String& msg);
  void callbackInfoTextC(const std_msgs::String& msg);

  // Ui::MyPluginWidget ui_; 
  QWidget* widget_;
  QProgressBar* _co2bar_fast;
  QProgressBar* _co2bar_slow;
  QLineEdit* _info_text_a;
  QLineEdit* _info_text_b;
  QLineEdit* _info_text_c;

#ifndef Q_MOC_RUN
  ros::NodeHandle _priv_nh;
  ros::NodeHandle _nh;

  ros::Subscriber _sub_co2_fast;
  ros::Subscriber _sub_co2_slow;
  ros::Subscriber _sub_info_a;
  ros::Subscriber _sub_info_b;
  ros::Subscriber _sub_info_c;

  ros::ServiceClient _srv_save_map;

#endif
};

} // namespace

#endif // my_namespace__my_plugin_H