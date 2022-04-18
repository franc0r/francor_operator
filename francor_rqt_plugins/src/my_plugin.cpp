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
#include <QLineEdit>

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

  // stop slam service
  auto stopSlamButton = new QPushButton("STOP SLAM");
  stopSlamButton->setMinimumHeight(64);
  layout->addWidget(stopSlamButton);
  this->connect(stopSlamButton, SIGNAL(pressed()), this, SLOT(stopSlam()));

  // co2 sensor value bars
  auto textBarFast = new QLabel("CO2 Sensor (fast)");
  layout->addWidget(textBarFast);
  _co2bar_fast = new QProgressBar;
  _co2bar_fast->setMaximum(50);
  _co2bar_fast->setMinimum(15);
  // _co2bar_fast->setFormat("%v ppm");
  layout->addWidget(_co2bar_fast);

  auto textBarSlow = new QLabel("CO2 Sensor (slow)");
  layout->addWidget(textBarSlow);
  _co2bar_slow = new QProgressBar;
  _co2bar_slow->setMaximum(2048);
  _co2bar_slow->setMinimum(0);
  _co2bar_slow->setFormat("%v ppm");
  layout->addWidget(_co2bar_slow);

#ifndef Q_MOC_RUN
  // co2 sensor value subscriber
  _sub_co2_fast = _nh.subscribe("/sensor_co2_fast", 2, &MyPlugin::callbackCo2SensorFast, this);
  _sub_co2_slow = _nh.subscribe("/sensor_co2_slow", 2, &MyPlugin::callbackCo2SensorSlow, this);

  // info text subscriber
  _sub_info_a = _nh.subscribe("info_text_a", 2, &MyPlugin::callbackInfoTextA, this);
  _sub_info_b = _nh.subscribe("info_text_b", 2, &MyPlugin::callbackInfoTextB, this);
  _sub_info_c = _nh.subscribe("info_text_c", 2, &MyPlugin::callbackInfoTextC, this);

  _srv_save_map = _nh.serviceClient<std_srvs::Empty>("save_geotiff");
#endif

  // info texts
  auto textInfoTextA = new QLabel(_sub_info_a.getTopic().c_str());
  layout->addWidget(textInfoTextA);
  _info_text_a = new QLineEdit;
  _info_text_a->setReadOnly(true);
  layout->addWidget(_info_text_a);

  auto textInfoTextB = new QLabel(_sub_info_b.getTopic().c_str());
  layout->addWidget(textInfoTextB);
  _info_text_b = new QLineEdit;
  _info_text_b->setReadOnly(true);
  layout->addWidget(_info_text_b);

  auto textInfoTextC = new QLabel(_sub_info_c.getTopic().c_str());
  layout->addWidget(textInfoTextC);
  _info_text_c = new QLineEdit;
  _info_text_c->setReadOnly(true);
  layout->addWidget(_info_text_c);
  
  // set layout and show final configured widget
  widget_->setLayout(layout);
  widget_->show();

  // start timer with a period of 100 ms
  auto timer = new QTimer(this);
  this->connect(timer, SIGNAL(timeout()), this, SLOT(process()));
  timer->start(100);
}

void MyPlugin::stop(void)
{
  ROS_INFO("Emergency Stop! Kill drive node!");
  QProcess::execute("rosnode", QStringList() << "kill" << "francor_drives_node");
}

void MyPlugin::stopSlam(void)
{

  std_srvs::Empty srv;
  if(!_srv_save_map.call(srv))
  {
    ROS_ERROR("Unable to call _srv_set_manipulator_axis_mode ...");
  }

  ROS_INFO("Stop SLAM by killing it.");
  QProcess::execute("rosnode", QStringList() << "kill" << "/slam_node");
  QProcess::execute("rosnode", QStringList() << "kill" << "/francor_victim_node");


}

void MyPlugin::process(void)
{
  ros::spinOnce();
}

#ifndef Q_MOC_RUN
void MyPlugin::callbackCo2SensorFast(const std_msgs::Int32& msg)
{
  ROS_INFO_STREAM("co2 sensor (fast): " << msg.data);
  _co2bar_fast->setValue(msg.data);
  // _co2bar_fast->update();
  //widget_->update();
}

void MyPlugin::callbackCo2SensorSlow(const std_msgs::Int32& msg)
{
  ROS_INFO_STREAM("c02 sensor (slow): " << msg.data << " ppm");
  // necessary to convert value correctly
  _co2bar_slow->setValue(msg.data * 2.0);
  // _co2bar_slow->update();
  // widget_->update();
}

void MyPlugin::callbackInfoTextA(const std_msgs::String& msg)
{
  _info_text_a->setText(msg.data.c_str());
}

void MyPlugin::callbackInfoTextB(const std_msgs::String& msg)
{
  _info_text_b->setText(msg.data.c_str());
}

void MyPlugin::callbackInfoTextC(const std_msgs::String& msg)
{
  _info_text_c->setText(msg.data.c_str());  
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