#ifndef my_namespace__my_plugin_H
#define my_namespace__my_plugin_H
 
// #include <rqt_gui_cpp/plugin.h>
// #include <ui_my_plugin.h>
#include <QWidget>

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <std_msgs/Int32.h>
#endif

class QProgressBar;

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
  void process(void);

private: 
  void callbackCo2Sensor(const std_msgs::Int32& msg);

  // Ui::MyPluginWidget ui_; 
  QWidget* widget_;
  QProgressBar* _co2bar;

#ifndef Q_MOC_RUN
  ros::NodeHandle _priv_nh;
  ros::NodeHandle _nh;

  ros::Subscriber _sub_co2;
#endif
};

} // namespace

#endif // my_namespace__my_plugin_H