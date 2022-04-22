/**
 * @file   FrancorJoy2Vel.cpp
 * @author Michael Schmidpeter
 * @date   2018-03-12
 * @brief  todo
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */

#include "francor_joy2vel/FrancorJoy2Vel.h"
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <topic_tools_interfaces/srv/detail/mux_select__struct.hpp>

FrancorJoy2Vel::FrancorJoy2Vel() : rclcpp::Node("francor_joy2vel_node")
{
  //rosParam
  // ros::NodeHandle privNh("~");
  // std::string string_val;
  // double      double_val;
  // int         int_val;
  // bool        bool_val;
  std::string joy_map;
  double rate;
  double max_lin_vel;
  double max_ang_vel;
  // double max_sh_vel;
  double dead_zone_sh;

  double axis_factor;
  double axis_offset;

  // int pan_default;
  double tilt_default;

  std::string front_cam_topic;
  std::string back_cam_topic;

  // privNh.param(         "joy_map" ,        joy_map,   std::string("default"));
  this->declare_parameter<std::string>("joy_map", std::string("default"));
  joy_map = this->get_parameter("joy_map").as_string();

  // privNh.param<double>( "rate" ,           rate,          50.0);
  this->declare_parameter<double>("rate", 50.0);
  rate = this->get_parameter("rate").as_double();

  // privNh.param<double>( "max_lin_vel" ,    max_lin_vel,   1.0);
  this->declare_parameter<double>("max_lin_vel", 1.0);
  max_lin_vel = this->get_parameter("max_lin_vel").as_double();

  // privNh.param<double>( "max_ang_vel" ,    max_ang_vel,   1.0);
  this->declare_parameter<double>("max_ang_vel", 1.0);
  max_ang_vel = this->get_parameter("max_ang_vel").as_double();

  // privNh.param<double>( "max_sh_vel" ,     max_sh_vel,   30.0);
  
  //privNh.param<double>( "dead_zone_sh" ,   dead_zone_sh,  0.3);
  this->declare_parameter<double>("dead_zone_sh", 0.3);
  dead_zone_sh = this->get_parameter("dead_zone_sh").as_double();

  //privNh.param<double>( "axis_factor" ,    axis_factor,   1600.0);
  this->declare_parameter<double>("axis_factor", 1600.0);
  axis_factor = this->get_parameter("axis_factor").as_double();

  // privNh.param<double>( "axis_offset" ,    axis_offset,   700.0);
  this->declare_parameter<double>("axis_offset", 700.0);
  axis_offset = this->get_parameter("axis_offset").as_double();

  // privNh.param<int>(    "pan_default"    ,    pan_default   ,   0);

  // privNh.param<double>(    "tilt_default"    ,   tilt_default   ,   0.5);
  this->declare_parameter<double>("tilt_default", 0.5);
  tilt_default = this->get_parameter("tilt_default").as_double();



  // privNh.param(         "front_cam_topic" ,   front_cam_topic,   std::string("/front_cam/image_raw/compressed"));
  this->declare_parameter<std::string>("front_cam_topic", std::string("/front_cam/image_raw/compressed"));
  front_cam_topic = this->get_parameter("front_cam_topic").as_string();

  // privNh.param(         "back_cam_topic" ,    back_cam_topic,   std::string("/back_cam/image_raw/compressed"));
  this->declare_parameter<std::string>("back_cam_topic", std::string("/back_cam/image_raw/compressed"));
  back_cam_topic = this->get_parameter("back_cam_topic").as_string();
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);

  //print all params
  RCLCPP_INFO(this->get_logger(), "############################################################");
  RCLCPP_INFO(this->get_logger(), "## francor_joy2vel_node - Params ##");
  RCLCPP_INFO(this->get_logger(), "############################################################");
  RCLCPP_INFO(this->get_logger(), "joy_map:         %s", joy_map.c_str());
  RCLCPP_INFO(this->get_logger(), "rate:            %f", rate);
  RCLCPP_INFO(this->get_logger(), "max_lin_vel:     %f", max_lin_vel);
  RCLCPP_INFO(this->get_logger(), "max_ang_vel:     %f", max_ang_vel);
  RCLCPP_INFO(this->get_logger(), "dead_zone_sh:    %f", dead_zone_sh);
  RCLCPP_INFO(this->get_logger(), "axis_factor:     %f", axis_factor);
  RCLCPP_INFO(this->get_logger(), "axis_offset:     %f", axis_offset);
  RCLCPP_INFO(this->get_logger(), "tilt_default:    %f", tilt_default);
  RCLCPP_INFO(this->get_logger(), "front_cam_topic: %s", front_cam_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "back_cam_topic:  %s", back_cam_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "############################################################");



  _max_lin_vel = max_lin_vel;
  _max_ang_vel = max_ang_vel;
  // _max_sh_vel  = max_sh_vel;

  // _sh_default.pan  = pan_default;
  // _sh_default.tilt = tilt_default;
  _sh_tilt_default = tilt_default;

  _axis_factor = axis_factor;
  _axis_offset = axis_offset;

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
    // ROS_INFO("Create Steamcontroller mapper");
    RCLCPP_INFO(this->get_logger(), "Create Steamcontroller mapper");

    _joy_mapper = std::make_unique<francor::JoyMapSc>(dead_zone_sh);
  }
  else if(joy_map == "todo")
  {
    // ROS_INFO("Create TODO mapper");
    RCLCPP_INFO(this->get_logger(), "Create TODO mapper");
    _joy_mapper = std::make_unique<francor::JoyMapSc>(dead_zone_sh);
  }
  else
  {
    // ROS_INFO("Create Default mapper (Ps4)");
    RCLCPP_INFO(this->get_logger(), "Create Default mapper (Ps4)");
    _joy_mapper = std::make_unique<francor::JoyMapPs4>(dead_zone_sh);
  }

  _joy_mapper->showInitMsg();

  _joy_mapper->attach_callback(francor::btn::TR_L,  std::bind(&FrancorJoy2Vel::btn_trigger_left_pressed, this));
  _joy_mapper->attach_callback(francor::btn::TR_R,  std::bind(&FrancorJoy2Vel::btn_trigger_right_pressed, this));
  _joy_mapper->attach_callback(francor::btn::X,     std::bind(&FrancorJoy2Vel::btn_x_pressed, this));
  _joy_mapper->attach_callback(francor::btn::Y,     std::bind(&FrancorJoy2Vel::btn_y_pressed, this));
  _joy_mapper->attach_callback(francor::btn::A,     std::bind(&FrancorJoy2Vel::btn_a_pressed, this));
  _joy_mapper->attach_callback(francor::btn::B,     std::bind(&FrancorJoy2Vel::btn_b_pressed, this));
  _joy_mapper->attach_callback(francor::btn::JS_L,  std::bind(&FrancorJoy2Vel::btn_joystick_left_pressed, this));
  _joy_mapper->attach_callback(francor::btn::JS_R,  std::bind(&FrancorJoy2Vel::btn_joystick_right_pressed, this));
  _joy_mapper->attach_callback(francor::btn::UP,    std::bind(&FrancorJoy2Vel::btn_up_pressed, this));
  _joy_mapper->attach_callback(francor::btn::SHARE, std::bind(&FrancorJoy2Vel::btn_share_pressed, this));

  //init publisher
  // _pubMode                    = _nh.advertise<std_msgs::String>           ("joy2vel/mode", 1);
  _pubMode = this->create_publisher<std_msgs::msg::String>("joy2vel/mode", 1);
  // _pubTwistStamped            = _nh.advertise<geometry_msgs::TwistStamped>("cmd_vel/stamped", 1);
  _pubTwistStamped = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel/stamped", 1);
  // _pubTwist                   = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  _pubTwist = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  // _pubServoPanSpeed           = _nh.advertise<std_msgs::Float64>("/servo_lx16a/sensor_head_yaw/speed",1);
  _pubServoPanSpeed = this->create_publisher<std_msgs::msg::Float64>("/servo_lx16a/sensor_head_yaw/speed", 1);
  // _pubServoTiltSpeed          = _nh.advertise<std_msgs::Float64>("/servo_lx16a/sensor_head_pitch/speed",1);
  _pubServoTiltSpeed = this->create_publisher<std_msgs::msg::Float64>("/servo_lx16a/sensor_head_pitch/speed", 1);
  // _pubServoPanPos             = _nh.advertise<std_msgs::Float64>("/servo_lx16a/sensor_head_yaw/pos",1);
  _pubServoPanPos = this->create_publisher<std_msgs::msg::Float64>("/servo_lx16a/sensor_head_yaw/pos", 1);
  // _pubServoTiltPos            = _nh.advertise<std_msgs::Float64>("/servo_lx16a/sensor_head_pitch/pos",1);
  _pubServoTiltPos = this->create_publisher<std_msgs::msg::Float64>("/servo_lx16a/sensor_head_pitch/pos", 1);
  // _pubManipulatorAxisSpeed    = _nh.advertise<francor_msgs::ManipulatorCmd>("manipulator/speed/axis", 1);
  _pubManipulatorAxisSpeed = this->create_publisher<francor_msgs::msg::ManipulatorCmd>("manipulator/speed/axis", 1);
  // _pubManipulaotrInverseSpeed = _nh.advertise<geometry_msgs::Vector3>("manipulator/speed/inverse", 1);
  _pubManipulaotrInverseSpeed = this->create_publisher<geometry_msgs::msg::Vector3>("manipulator/speed/inverse", 1);
  // _pubAddVictim               = _nh.advertise<std_msgs::Bool>             ("francor/add_victim", 1);
  _pubAddVictim = this->create_publisher<std_msgs::msg::Bool>("francor/add_victim", 1);


  //inti subscriber
  // _subJoy           = _nh.subscribe("joy", 1, &FrancorJoy2Vel::subJoy_callback, this);
  _subJoy = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&FrancorJoy2Vel::subJoy_callback, this, std::placeholders::_1));
  // _subDiagonstics   = _nh.subscribe("/diagnostics", 1, &FrancorJoy2Vel::subDiagnostic_callback, this);
  _subDiagnostics = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10, std::bind(&FrancorJoy2Vel::subDiagnostic_callback, this, std::placeholders::_1));

  // _srv_robotic_arm_stand_by = _nh.serviceClient<std_srvs::Empty>("/robotic_arm/set_stand_by");
  // _srv_robotic_arm_active   = _nh.serviceClient<std_srvs::Empty>("/robotic_arm/set_active");

  // _srv_sw_drive_image               = _nh.serviceClient<topic_tools::MuxSelect>("/mux/select");
  _srv_sw_drive_image = this->create_client<topic_tools_interfaces::srv::MuxSelect>("/mux/select");
  // _srv_set_manipulator_axis_mode    = _nh.serviceClient<std_srvs::Empty>("manipulator/set_mode/axis");
  _srv_set_manipulator_axis_mode = this->create_client<std_srvs::srv::Empty>("manipulator/set_mode/axis");
  // _srv_set_manipulator_inverse_mode = _nh.serviceClient<std_srvs::Empty>("manipulator/set_mode/inverse");
  _srv_set_manipulator_inverse_mode = this->create_client<std_srvs::srv::Empty>("manipulator/set_mode/inverse");


  _reverse_drive = false;

  _twist_enabled = true;

  _mode = DRIVE;
}

FrancorJoy2Vel::~FrancorJoy2Vel()
{ }

void FrancorJoy2Vel::init()
{
  //create timer
  // _loopTimer     = _nh.createTimer(ros::Duration(1 / _rate), &FrancorJoy2Vel::loop_callback, this);
  _loopTimer = this->create_wall_timer(std::chrono::duration<double>(1/_rate), std::bind(&FrancorJoy2Vel::loop_callback, this));
  // _loopModeTimer = _nh.createTimer(ros::Duration(0.1), &FrancorJoy2Vel::loop_mode_callback, this);
  _loopModeTimer = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&FrancorJoy2Vel::loop_mode_callback, this));
  // this->run();
}

// void FrancorJoy2Vel::run()
// {
//   ros::spin();
// }

void FrancorJoy2Vel::subJoy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  _first_msg_rdy = true;
  _joy = msg;
}

void FrancorJoy2Vel::subDiagnostic_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  for(const auto& e : msg->status)
  {
    //ignore all others //todo fix node name ...
    if(e.name == "joy_node: Joystick Driver Status")
    {
      // std::cout << "got diagonstic info" << std::endl;
      
      //check state of joystick via message if no "OK" then error and reinit is needed
      if(e.message != "OK")
      {
        if(_joystick_rdy)
        {
          //once info
          // ROS_INFO_STREAM("Detatched Joystick... plz enable Joystick");
          RCLCPP_INFO(this->get_logger(), "Detatched Joystick... plz enable Joystick");
          _joy_mapper->showInitMsg();
        }
        _joystick_rdy = false;
      }
      break;
    }
  }
}

void FrancorJoy2Vel::loop_callback()
{
  if(!_first_msg_rdy)
  {
    return;
  }
  _joy_mapper->map(*_joy);
  //prove init
  if(!_joystick_rdy)
  {
    auto joy_input = _joy_mapper->getJoyInput();
    if(joy_input.init_ok)
    {
      _joystick_rdy = true;
      // ROS_INFO("Joystick initialized...");
      RCLCPP_INFO(this->get_logger(), "Joystick initialized...");
    }
    //pub empty twist
    if(_twist_enabled)
    {
      _pubTwistStamped->publish(this->getEmptyTwist());
      _pubTwist->publish(this->getEmptyTwist().twist);
    }
    // _pubDriveAction.publish(this->toDriveAction(DRIVE_ACTION_NONE));
    return;
  }

  //for buttons
  _joy_mapper->triggerBtn_callbacks();  

  if(_mode == DRIVE)
  {
    //old sh
    // _pubSpeedSensorHead.publish(_joy_mapper->toSensorHeadCmd(_max_sh_vel, _reverse_drive));
    //new sh
    std_msgs::msg::Float64 msg;
    auto speed = _joy_mapper->toSensorHeadSpeed(_reverse_drive);
    msg.data = speed.first;
    _pubServoPanSpeed->publish(msg);
    msg.data = speed.second;
    _pubServoTiltSpeed->publish(msg);

    if(_twist_enabled)
    {
      _pubTwistStamped->publish(_joy_mapper->toTwistStamped(_max_lin_vel, _max_ang_vel, _reverse_drive));
      _pubTwist->publish(_joy_mapper->toTwistStamped(_max_lin_vel, _max_ang_vel, _reverse_drive).twist);
    }
  }
  else if(_mode == MANIPULATE_DIRECT)
  {
    _pubManipulatorAxisSpeed->publish(_joy_mapper->toManipulatorCmd(false));

    //todo may dont do this -> add extra mode for drivin while manipulating
    _pubTwistStamped->publish(this->getEmptyTwist());
    _pubTwist->publish(this->getEmptyTwist().twist);
  }
  else if(_mode == MANIPULATE_INVERSE)
  {
    _pubManipulatorAxisSpeed->publish(_joy_mapper->toManipulatorCmd(true));
    _pubManipulaotrInverseSpeed->publish(_joy_mapper->toManipulatorCmd_inverse());

    //todo may dont do this -> add extra mode for drivin while manipulating
    _pubTwistStamped->publish(this->getEmptyTwist());
    _pubTwist->publish(this->getEmptyTwist().twist);
  }

}

void FrancorJoy2Vel::loop_mode_callback()
{
  std_msgs::msg::String msg;
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
  _pubMode->publish(msg);
}








