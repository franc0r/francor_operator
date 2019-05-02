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

namespace my_namespace
{

MyPlugin::MyPlugin()
  : widget_(0),
    _priv_nh("~"),
    _nh()
{
  widget_ = new QWidget();

  auto layout = new QGridLayout;
  auto stopButton = new QPushButton(QIcon("/home/knueppl/ros/francor/src/francor_operator/francor_rqt_plugins/icon/icon-stop.png"), "");
  stopButton->setMinimumSize(256, 256);
  layout->addWidget(stopButton);
  this->connect(stopButton, SIGNAL(pressed()), this, SLOT(stop()));

  _co2bar = new QProgressBar;
  _co2bar->setMaximum(1024);
  _co2bar->setMinimum(0);
  layout->addWidget(_co2bar);

  widget_->setLayout(layout);
  widget_->show();

#ifndef Q_MOC_RUN
  _sub_co2 = _nh.subscribe("/sensor_co2/fast", 2, &MyPlugin::callbackCo2Sensor, this);
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
void MyPlugin::callbackCo2Sensor(const std_msgs::Int32& msg)
{
  std::cout << "co2 value = " << msg.data << std::endl;
  _co2bar->setValue(msg.data);
  widget_->update();
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