/**
 * THIS IS A FUCKTING HACK AND ONLY FOR ROBOCUP USE!
 * 
 * \author Christian Merkl (knueppl@gmx.de)
 * \date 2. May 2019
 */
#include "my_plugin.h"

#include <ros/ros.h>
// #include <pluginlib/class_list_macros.h>
// #include <QStringList>
#include <QApplication>
#include <QPushButton>
#include <QGridLayout>
#include <QProcess>

namespace my_namespace
{

MyPlugin::MyPlugin()
  : widget_(0)
{
  widget_ = new QWidget();

  auto layout = new QGridLayout;
  auto stopButton = new QPushButton(QIcon("/home/knueppl/ros/francor/src/francor_operator/francor_rqt_plugins/icon/icon-stop.png"), "");
  stopButton->setMinimumSize(256, 256);
  layout->addWidget(stopButton);
  this->connect(stopButton, SIGNAL(pressed()), this, SLOT(stop()));

  widget_->setLayout(layout);
  widget_->show();
}

void MyPlugin::stop(void)
{
  ROS_FATAL("Emergency Stop! Kill drive node!");
  QProcess::execute("rosnode", QStringList() << "kill" << "\francor_drivers_node");
}

} // end namespace 

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  ros::init(argc, argv, "gui_for_schmiddey");

  my_namespace::MyPlugin plugin;

  return app.exec();
}