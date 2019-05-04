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

  std::string front_cam_topic;
  std::string back_cam_topic;

  privNh.param(         "joy_map" ,        joy_map,   std::string("default"));
  privNh.param<double>( "rate" ,           rate,          50.0);
  privNh.param<double>( "max_lin_vel" ,    max_lin_vel,   1.0);
  privNh.param<double>( "max_ang_vel" ,    max_ang_vel,   1.0);
  privNh.param<double>( "max_sh_vel" ,     max_sh_vel,   30.0);
  privNh.param<double>( "dead_zone_sh" ,   dead_zone_sh,  0.3);

  privNh.param<int>(    "pan_default"    ,    pan_default   ,   0);
  privNh.param<int>(    "tilt_default"    ,   tilt_default   ,   20);

  privNh.param(         "front_cam_topic" ,   front_cam_topic,   std::string("/front_cam/image_raw/compressed"));
  privNh.param(         "back_cam_topic" ,    back_cam_topic,   std::string("/back_cam/image_raw/compressed"));
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);

  _max_lin_vel = max_lin_vel;
  _max_ang_vel = max_ang_vel;
  _max_sh_vel  = max_sh_vel;

  _sh_default.pan  = pan_default;
  _sh_default.tilt = tilt_default;

  _front_cam_topic = front_cam_topic;
  _back_cam_topic  = back_cam_topic;

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
    ROS_INFO("Create Default mapper (Ps4)");
    _joy_mapper = std::make_unique<francor::JoyMapPs4>(dead_zone_sh);
  }

  _joy_mapper->showInitMsg();

  _joy_mapper->attach_callback(francor::btn::TR_L, std::bind(&FrancorJoy2Vel::btn_trigger_left_pressed, this));
  _joy_mapper->attach_callback(francor::btn::TR_R, std::bind(&FrancorJoy2Vel::btn_trigger_right_pressed, this));
  _joy_mapper->attach_callback(francor::btn::X,    std::bind(&FrancorJoy2Vel::btn_x_pressed, this));
  _joy_mapper->attach_callback(francor::btn::Y,    std::bind(&FrancorJoy2Vel::btn_y_pressed, this));
  _joy_mapper->attach_callback(francor::btn::A,    std::bind(&FrancorJoy2Vel::btn_a_pressed, this));
  _joy_mapper->attach_callback(francor::btn::B,    std::bind(&FrancorJoy2Vel::btn_b_pressed, this));
  _joy_mapper->attach_callback(francor::btn::JS_L, std::bind(&FrancorJoy2Vel::btn_joystick_left_pressed, this));
  _joy_mapper->attach_callback(francor::btn::JS_R, std::bind(&FrancorJoy2Vel::btn_joystick_right_pressed, this));
  _joy_mapper->attach_callback(francor::btn::UP,   std::bind(&FrancorJoy2Vel::btn_up_pressed, this));

  //init publisher
  _pubMode            = _nh.advertise<std_msgs::String>           ("joy2vel/mode", 1);
  _pubTwistStamped    = _nh.advertise<geometry_msgs::TwistStamped>("cmd_vel/stamped", 1);
  _pubTwist           = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  _pubSpeedSensorHead = _nh.advertise<francor_msgs::SensorHeadCmd>("/sensor_head/set_speed", 1);
  _pubPosSensorHead   = _nh.advertise<francor_msgs::SensorHeadCmd>("/sensor_head/set_pos", 1);
  _pubRoboticArm      = _nh.advertise<std_msgs::Float64MultiArray>("robotic_arm/set_joint_speed", 1);
  _pubDriveAction     = _nh.advertise<std_msgs::String>           ("drive/action", 1);
  _pubAddVictim       = _nh.advertise<std_msgs::Bool>             ("francor/add_victim", 1);
  //inti subscriber
  _subJoy           = _nh.subscribe("joy", 1, &FrancorJoy2Vel::subJoy_callback, this);
  _subDiagonstics   = _nh.subscribe("/diagnostics", 1, &FrancorJoy2Vel::subDiagnostic_callback, this);

  _srv_robotic_arm_stand_by = _nh.serviceClient<std_srvs::Empty>("/robotic_arm/set_stand_by");
  _srv_robotic_arm_active   = _nh.serviceClient<std_srvs::Empty>("/robotic_arm/set_active");

  _srv_sw_drive_image       = _nh.serviceClient<topic_tools::MuxSelect>("/mux/select");

  _reverse_drive = false;

  _twist_enabled = true;

  _mode = DRIVE;
}

FrancorJoy2Vel::~FrancorJoy2Vel()
{ }

void FrancorJoy2Vel::start()
{
  //create timer
  _loopTimer     = _nh.createTimer(ros::Duration(1 / _rate), &FrancorJoy2Vel::loop_callback, this);
  _loopModeTimer = _nh.createTimer(ros::Duration(0.1), &FrancorJoy2Vel::loop_mode_callback, this);
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
    if(_twist_enabled)
    {
      _pubTwistStamped.publish(this->getEmptyTwist());
      _pubTwist.publish(this->getEmptyTwist().twist);
    }
    // _pubDriveAction.publish(this->toDriveAction(DRIVE_ACTION_NONE));
    return;
  }

  //drive action
  // if(_mode == DRIVE)
  // {
  //   if(_joy_mapper->getJoyInput().btns[francor::btn::A])
  //   {
  //     _pubDriveAction.publish(this->toDriveAction(DRIVE_ACTION_CLIMP));
  //   }
  //   else if(_joy_mapper->getJoyInput().btns[francor::btn::X])
  //   {
  //     _pubDriveAction.publish(this->toDriveAction(DRIVE_ACTION_BOGIE_UP_BOOSTED));
  //   }
  //   else if(_joy_mapper->getJoyInput().btns[francor::btn::Y])
  //   {
  //     //prove if moving
  //     if(_joy_mapper->getJoyInput().vel_ang == 0.0 && _joy_mapper->getJoyInput().vel_lin_x == 0.0) 
  //     {//no movement
  //       _pubDriveAction.publish(this->toDriveAction(DRIVE_ACTION_BOGIE_UP));
  //     }
  //     else
  //     {//movement
  //       _pubDriveAction.publish(this->toDriveAction(DRIVE_ACTION_BOGIE_UP_DRIVE));
  //     }
  //   }
  //   else
  //   {
  //     _pubDriveAction.publish(this->toDriveAction(DRIVE_ACTION_NONE));
  //   }
    
  // }

  //for buttons
  _joy_mapper->triggerBtn_callbacks();  

  if(_mode == DRIVE)
  {
    _pubSpeedSensorHead.publish(_joy_mapper->toSensorHeadCmd(_max_sh_vel, _reverse_drive));
    if(_twist_enabled)
    {
      _pubTwistStamped.publish(_joy_mapper->toTwistStamped(_max_lin_vel, _max_ang_vel, _reverse_drive));
      _pubTwist.publish(_joy_mapper->toTwistStamped(_max_lin_vel, _max_ang_vel, _reverse_drive).twist);
    }
  }
  else if(_mode == MANIPULATE_DIRECT)
  {
    _pubRoboticArm.publish(_joy_mapper->toRoboicArmCmd());
    _pubTwistStamped.publish(this->getEmptyTwist());
    _pubTwist.publish(this->getEmptyTwist().twist);
  }
  else if(_mode == MANIPULATE_INVERSE)
  {

    _pubTwistStamped.publish(this->getEmptyTwist());
    _pubTwist.publish(this->getEmptyTwist().twist);
  }

}

void FrancorJoy2Vel::loop_mode_callback(const ros::TimerEvent& e)
{
  std_msgs::String msg;
  if(!_joystick_rdy)
  {
    msg.data = "DETACHED_plz_reinit";
  }
  else if(_mode == DRIVE)
  {
    if(_twist_enabled)
    {
      msg.data = "DRIVE - ENABLED";
    }
    else
    {
      msg.data = "DRIVE - DISABLED";
    }
    
  }
  else if(_mode == MANIPULATE_DIRECT)
  {
    msg.data = "MANIPULATE_DIRECT";
  }
  else if(_mode == MANIPULATE_INVERSE)
  {
    msg.data = "MANIPULATE_INVERSE";
  }
  else
  {
    msg.data = "INVALID";
  }
  _pubMode.publish(msg);
}








// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "francor_joy2vel_node");
    ros::NodeHandle nh("~");

    FrancorJoy2Vel node;
    node.start();

}
