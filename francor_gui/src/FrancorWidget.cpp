#include "FrancorWidget.hpp"

#include "FrancorGuiNode.hpp"

namespace francor{



FrancorWidget::FrancorWidget(MainWindow* parent)
{
  _parent = parent;
  _grid_layout = new QGridLayout;

  //btns for vid
  _btn_vid_disable = new QPushButton("vid_disable");
  _btn_vid_manip = new QPushButton("vid_manip");
  _btn_vid_haz = new QPushButton("vid_haz");
  _grid_layout->addWidget(_btn_vid_disable);
  _grid_layout->addWidget(_btn_vid_manip);
  _grid_layout->addWidget(_btn_vid_haz);

  this->connect(_btn_vid_disable, SIGNAL(clicked()), _parent, SLOT(btn_vid_disable_clicked()));
  this->connect(_btn_vid_manip, SIGNAL(clicked()), _parent, SLOT(btn_vid_manip_clicked()));
  this->connect(_btn_vid_haz, SIGNAL(clicked()), _parent, SLOT(btn_vid_haz_clicked()));
  

  auto stopButton = new QPushButton(QIcon("/home/m1ch1/workspace/ros2/dev_ws/src/francor_operator/francor_gui/icon/icon-stop.png"), "");
  stopButton->setMinimumSize(256, 256);
  _grid_layout->addWidget(stopButton);
  // this->connect(stopButton, SIGNAL(pressed()), this, SLOT(stop()));
  



  // stop slam service
  auto stopSlamButton = new QPushButton("STOP SLAM");
  stopSlamButton->setMinimumHeight(64);
  _grid_layout->addWidget(stopSlamButton);
  // this->connect(stopSlamButton, SIGNAL(pressed()), this, SLOT(stopSlam()));

  // co2 sensor value bars
  auto textBarFast = new QLabel("CO2 Sensor (fast)");
  _grid_layout->addWidget(textBarFast);
  _co2bar_fast = new QProgressBar;
  _co2bar_fast->setMaximum(50);
  _co2bar_fast->setMinimum(15);
  // _co2bar_fast->setFormat("%v ppm");
  _grid_layout->addWidget(_co2bar_fast);

  auto textBarSlow = new QLabel("CO2 Sensor (slow)");
  _grid_layout->addWidget(textBarSlow);
  _co2bar_slow = new QProgressBar;
  _co2bar_slow->setMaximum(100);
  _co2bar_slow->setMinimum(0);
  _co2bar_slow->setFormat("%");
  _grid_layout->addWidget(_co2bar_slow);

    // info texts
  auto textInfoTextA = new QLabel("Info Text A");
  _grid_layout->addWidget(textInfoTextA);
  _info_text_a = new QLineEdit;
  _info_text_a->setReadOnly(true);
  _grid_layout->addWidget(_info_text_a);

  auto textInfoTextB = new QLabel("Info Text B");
  _grid_layout->addWidget(textInfoTextB);
  _info_text_b = new QLineEdit;
  _info_text_b->setReadOnly(true);
  _grid_layout->addWidget(_info_text_b);

  auto textInfoTextC = new QLabel("Info Text C");
  _grid_layout->addWidget(textInfoTextC);
  _info_text_c = new QLineEdit;
  _info_text_c->setReadOnly(true);
  _grid_layout->addWidget(_info_text_c);
  

  // auto temp_grid_layout = new QGridLayout;
  

  _lbl_temp_front_right = new QLabel("temp_front_right");
  _lbl_temp_front_left  = new QLabel("temp_front_left");  
  _lbl_temp_back_left   = new QLabel("temp_back_left");
  _lbl_temp_back_right  = new QLabel("temp_back_right");

  _grid_layout->addWidget(_lbl_temp_front_right);
  _grid_layout->addWidget(_lbl_temp_front_left);
  _grid_layout->addWidget(_lbl_temp_back_left);
  _grid_layout->addWidget(_lbl_temp_back_right);

  // _grid_layout->addItem(temp_grid_layout,0,0);
}

void FrancorWidget::setCo2Value(int value)
{
  _co2bar_slow->setValue(value);
} 

} //namespace francor
