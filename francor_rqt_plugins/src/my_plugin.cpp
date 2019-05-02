/**
 * THIS IS A FUCKTING HACK AND ONLY FOR ROBOCUP USE!
 * 
 * \author Christian Merkl (knueppl@gmx.de)
 * \date 2. May 2019
 */
#include "my_plugin.h"

// #include <pluginlib/class_list_macros.h>
// #include <QStringList>
#include <QApplication>
#include <QPushButton>
#include <QGridLayout>
#include <QProcess>
#include <QProgressBar>
#include <QTimer>
#include <QLabel>

namespace my_namespace
{

MyPlugin::MyPlugin()
  : widget_(0),
    _priv_nh("~"),
    _nh()
{
  widget_ = new QWidget();

  auto layout = new QGridLayout;
  auto stopButton = new QPushButton(QIcon("/home/m1ch1/workspace/ros/catkin_ws/src/francor_operator/francor_rqt_plugins/icon/icon-stop.png"), "");
  stopButton->setMinimumSize(256, 256);
  layout->addWidget(stopButton);
  this->connect(stopButton, SIGNAL(pressed()), this, SLOT(stop()));

  // co2 sensor value bars
  auto textBarFast = new QLabel("CO2 Sensor (fast)");
  layout->addWidget(textBarFast);
  _co2bar_fast = new QProgressBar;
  _co2bar_fast->setMaximum(20);
  _co2bar_fast->setMinimum(0);
  layout->addWidget(_co2bar_fast);

  auto textBarSlow = new QLabel("CO2 Sensor (slow)");
  layout->addWidget(textBarSlow);
  _co2bar_slow = new QProgressBar;
  _co2bar_slow->setMaximum(1024);
  _co2bar_slow->setMinimum(0);
  layout->addWidget(_co2bar_slow);

  widget_->setLayout(layout);
  widget_->show();

#ifndef Q_MOC_RUN
  _sub_co2_fast = _nh.subscribe("/sensor_co2_fast", 2, &MyPlugin::callbackCo2SensorFast, this);
  _sub_co2_slow = _nh.subscribe("/sensor_co2_slow", 2, &MyPlugin::callbackCo2SensorSlow, this);
#endif

  // start timer with a period of 100 ms
  auto timer = new QTimer(this);
  this->connect(timer, SIGNAL(timeout()), this, SLOT(process()));
  timer->start(100);
}

void MyPlugin::stop(void)
{
  ROS_FATAL("Emergency Stop! Kill drive node!");
  QProcess::execute("rosnode", QStringList() << "kill" << "francor_drives_node");
}

void MyPlugin::process(void)
{
  ros::spinOnce();
}

#ifndef Q_MOC_RUN
void MyPlugin::callbackCo2SensorFast(const std_msgs::Int32& msg)
{
  ROS_INFO_STREAM("fast: " << msg.data);
  _co2bar_fast->setValue(msg.data);
  // _co2bar_fast->update();
  //widget_->update();
}

void MyPlugin::callbackCo2SensorSlow(const std_msgs::Int32& msg)
{
  ROS_INFO_STREAM("slow: " << msg.data);
  _co2bar_slow->setValue(msg.data);
  // _co2bar_slow->update();
  // widget_->update();
}
#endif

} // end namespace 

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

#ifndef Q_MOC_RUN
  ros::init(argc, argv, "gui_for_schmiddey");
#endif

  my_namespace::MyPlugin plugin;

  return app.exec();
}