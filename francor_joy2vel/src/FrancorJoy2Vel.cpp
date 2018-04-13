/**
 * @file   FrancorJoy2Vel.cpp
 * @author Michael Schmidpeter
 * @date   2018-03-12
 * @brief  todo
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */

#include "FrancorJoy2Vel.h"

FrancorJoy2Vel::FrancorJoy2Vel()
{
  //rosParam
  ros::NodeHandle privNh("~");
  // std::string string_val;
  // double      double_val;
  // int         int_val;
  // bool        bool_val;
  std::string joy_map;
  double rate;
  double max_lin_vel;
  double max_ang_vel;
  double max_sh_vel;
  double dead_zone_sh;

  int pan_default;
  int tilt_default;

  privNh.param(         "joy_map" ,        joy_map,   std::string("default"));
  privNh.param<double>( "rate" ,           rate,          50.0);
  privNh.param<double>( "max_lin_vel" ,    max_lin_vel,   1.0);
  privNh.param<double>( "max_ang_vel" ,    max_ang_vel,   1.0);
  privNh.param<double>( "max_sh_vel" ,     max_sh_vel,   30.0);
  privNh.param<double>( "dead_zone_sh" ,   dead_zone_sh,  0.3);

  privNh.param<int>(    "pan_default"    ,    pan_default   ,   0);
  privNh.param<int>(    "tilt_default"    ,    tilt_default   ,   20);
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);

  _max_lin_vel = max_lin_vel;
  _max_ang_vel = max_ang_vel;
  _max_sh_vel  = max_sh_vel;

  _sh_default.pan  = pan_default;
  _sh_default.tilt = tilt_default;

  _rate = rate;
  if(rate < 1.0)
  {
    rate = 1.0;
  }

  //joy mapper
  if(joy_map == "sc")
  {
    ROS_INFO("Create Steamcontroller mapper");
    _joy_mapper = std::make_unique<francor::JoyMapSc>(dead_zone_sh);
  }
  else if(joy_map == "todo")
  {
    ROS_INFO("Create TODO mapper");
    _joy_mapper = std::make_unique<francor::JoyMapSc>(dead_zone_sh);
  }
  else
  {
    ROS_INFO("Create Default mapper (Steamcontroller)");
    _joy_mapper = std::make_unique<francor::JoyMapSc>(dead_zone_sh);
  }

  _joy_mapper->showInitMsg();

  _joy_mapper->attach_callback(francor::btn::TR_L, std::bind(&FrancorJoy2Vel::btn_trigger_left_pressed, this));
  _joy_mapper->attach_callback(francor::btn::TR_R, std::bind(&FrancorJoy2Vel::btn_trigger_right_pressed, this));
  _joy_mapper->attach_callback(francor::btn::X, std::bind(&FrancorJoy2Vel::btn_x_pressed, this));
  _joy_mapper->attach_callback(francor::btn::Y, std::bind(&FrancorJoy2Vel::btn_y_pressed, this));

  //init publisher
  _pubTwist           = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  _pubSpeedSensorHead = _nh.advertise<francor_msgs::SensorHeadCmd>("/sensor_head/set_speed", 1);
  _pubPosSensorHead   = _nh.advertise<francor_msgs::SensorHeadCmd>("/sensor_head/set_pos", 1);

  //inti subscriber
  _subJoy           = _nh.subscribe("joy", 1, &FrancorJoy2Vel::subJoy_callback, this);
  _subDiagonstics   = _nh.subscribe("/diagnostics", 1, &FrancorJoy2Vel::subDiagnostic_callback, this);
}

FrancorJoy2Vel::~FrancorJoy2Vel()
{ }

void FrancorJoy2Vel::start()
{
  //create timer
  _loopTimer = _nh.createTimer(ros::Duration(1 / _rate), &FrancorJoy2Vel::loop_callback, this);
  this->run();
}

void FrancorJoy2Vel::run()
{
  ros::spin();
}

void FrancorJoy2Vel::subJoy_callback(const sensor_msgs::Joy& msg)
{
  _first_msg_rdy = true;
  _joy = msg;
}

void FrancorJoy2Vel::subDiagnostic_callback(const diagnostic_msgs::DiagnosticArray& msg)
{
  for(const auto& e : msg.status)
  {
    //ignore all others
    if(e.name == "joy_node: Joystick Driver Status")
    {
      // std::cout << "got diagonstic info" << std::endl;
      
      //check state of joystick via message if no "OK" then error and reinit is needed
      if(e.message != "OK")
      {
        if(_joystick_rdy)
        {
          //once info
          ROS_INFO_STREAM("Detatched Joystick... plz enable Joystick");
          _joy_mapper->showInitMsg();
        }
        _joystick_rdy = false;
      }
      break;
    }
  }
}

void FrancorJoy2Vel::loop_callback(const ros::TimerEvent& e)
{
  if(!_first_msg_rdy)
  {
    return;
  }
  _joy_mapper->map(_joy);
  //prove init
  if(!_joystick_rdy)
  {
    auto joy_input = _joy_mapper->getJoyInput();
    if(joy_input.init_ok)
    {
      _joystick_rdy = true;
      ROS_INFO("Joystick initialized...");
    }
    //pub empty twist
    geometry_msgs::TwistStamped empty;
    empty.header.stamp = ros::Time::now();
    empty.twist.linear.x = 0.0;
    empty.twist.linear.y = 0.0;
    empty.twist.angular.z = 0.0;
    _pubTwist.publish(empty.twist);
    return;
  }
  
  //for buttons
  _joy_mapper->triggerBtn_callbacks();

  _pubSpeedSensorHead.publish(_joy_mapper->toSensorHeadCmd(_max_sh_vel));
  _pubTwist.publish(_joy_mapper->toTwistStamped(_max_lin_vel, _max_ang_vel).twist);
}










// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "francor_joy2vel_node");
    ros::NodeHandle nh("~");

    FrancorJoy2Vel node;
    node.start();

}
