/**
 * @file   FrancorJoy2Vel.h
 * @author Michael Schmidpeter
 * @date   2018-03-12
 * @brief  todo
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */

#ifndef FRANCORJOY2VEL_H_
#define FRANCORJOY2VEL_H_

#include <iostream>
#include <string>
#include <memory>
#include <functional>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <topic_tools/MuxSelect.h>

#include <francor_joy2vel/JoyMap.h>
#include <francor_joy2vel/JoyMapSc.h> //steam controller differential
#include <francor_joy2vel/JoyMapPs4.h>



#include <francor_msgs/SensorHeadCmd.h>

class FrancorJoy2Vel
{

public:
  enum enum_modes{
    DRIVE = 0,
    MANIPULATE_DIRECT,
    MANIPULATE_INVERSE
  };

  FrancorJoy2Vel();
  virtual ~FrancorJoy2Vel();

  /**
   *
   * @brief
   *
   * @return  void
   */
  void start();

private:    //functions

  /**
   *
   * @brief this function containts the main working loop
   *
   * @param[in,out]  void
   *
   * @return 		   void
   */
  void run();

  void loop_callback(const ros::TimerEvent& e);

  void loop_mode_callback(const ros::TimerEvent& e);

  void subJoy_callback(const sensor_msgs::Joy& msg);
  void subDiagnostic_callback(const diagnostic_msgs::DiagnosticArray& msg); //for detecting joystic timeout...

  geometry_msgs::TwistStamped getEmptyTwist() const
  {
    geometry_msgs::TwistStamped empty;
    empty.header.stamp = ros::Time::now();
    empty.twist.linear.x = 0.0;
    empty.twist.linear.y = 0.0;
    empty.twist.angular.z = 0.0;
    return empty;
  }

  francor_msgs::SensorHeadCmd getDefaultSensorHead()
  {
    francor_msgs::SensorHeadCmd cmd = _sh_default;
    if(_reverse_drive)
    {
      cmd.tilt *= -1;
    }
    return cmd;
  }


  std_msgs::String toDriveAction(const std::string& drive_action)
  {
    std_msgs::String msg;
    msg.data = drive_action;
    return msg;
  }

  void btn_trigger_left_pressed()
  {
    ROS_INFO("Trigger left");
    if(_mode == DRIVE)
    {
    //  _pubPosSensorHead.publish(_sh_default);
    }
    else if(_mode == MANIPULATE_DIRECT || _mode == MANIPULATE_INVERSE)
    {
      std_srvs::Empty srv;
      if(_srv_robotic_arm_active.call(srv))
      {
        ROS_INFO("Set arm active");
      }
      else
      {
        ROS_WARN("Unable to call set arm active Service");
      }
    }
  }
  void btn_trigger_right_pressed()
  {
    ROS_INFO("Trigger right");
    if(_mode == DRIVE)
    {
      _pubPosSensorHead.publish(this->getDefaultSensorHead());
    }
    else if(_mode == MANIPULATE_DIRECT || _mode == MANIPULATE_INVERSE)
    {
      std_srvs::Empty srv;
      if(_srv_robotic_arm_stand_by.call(srv))
      {
        ROS_INFO("Set arm stand by");
      }
      else
      {
        ROS_WARN("Unable to call set arm stand by Service");
      }
    }
  }
  void btn_x_pressed()
  {
    ROS_INFO("Button x");
    if(_mode == DRIVE)
    {
      //disable / enable Twist;
      if(_twist_enabled)
      {
        ROS_INFO("Disable Twist");
        _twist_enabled = false;
      }
      else
      {
        ROS_INFO("Enable Twist");
        _twist_enabled = true;
      }
    }
  }
  void btn_y_pressed()
  {
    ROS_INFO("Button y");
  }

  void btn_a_pressed()
  {
    ROS_INFO("Button a");
  }

  void btn_b_pressed()
  {
    ROS_INFO("Button b");
    std_msgs::Bool msg;
    msg.data = true;
    _pubAddVictim.publish(msg);
  }

  void btn_joystick_left_pressed()
  {
    ROS_INFO("Button JSL");
    //change mode
    if(_mode == DRIVE)
    {
      ROS_INFO("Set Mode: MANIPULATE_DIRECT");
      _mode = MANIPULATE_DIRECT;
    }
    else if(_mode == MANIPULATE_DIRECT)
    {
      ROS_INFO("Set Mode: MANIPULATE_INVERSE");
      _mode = MANIPULATE_INVERSE;
    }
    else if(_mode == MANIPULATE_INVERSE)
    {
      ROS_INFO("Set Mode: DRIVE");
      _mode = DRIVE;
    }
    else
    {
      //INVALID MODE -> go to DRIVE mode
      ROS_INFO("Invalid Mode -> Set Mode: DRIVE");
      _mode = DRIVE;
    }
  }

  void btn_joystick_right_pressed()
  {
    ROS_INFO("Button JSR");

  }

  void btn_up_pressed()
  {
    ROS_INFO("Button UP");
    //toggle reverse
    _reverse_drive = !_reverse_drive;
    //swich sensorhead to default
    _pubPosSensorHead.publish(this->getDefaultSensorHead());

    if(_reverse_drive)
    {
      //swich driver cam topic
      topic_tools::MuxSelect srv;
      srv.request.topic = _back_cam_topic;
      if(!_srv_sw_drive_image.call(srv))
      {
        ROS_WARN("Unable to call swich topic srv");
      }
    }
    else
    {
      //swich driver cam topic
      topic_tools::MuxSelect srv;
      srv.request.topic = _front_cam_topic;
      if(!_srv_sw_drive_image.call(srv))
      {
        ROS_WARN("Unable to call swich topic srv");
      }
    }
  }

private:    //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pubMode;
  ros::Publisher _pubTwist;
  ros::Publisher _pubTwistStamped;
  ros::Publisher _pubSpeedSensorHead;
  ros::Publisher _pubPosSensorHead;
  ros::Publisher _pubRoboticArm;
  ros::Publisher _pubDriveAction;
  ros::Publisher _pubAddVictim;


  //for testing stuff
  ros::Publisher _pubAxis0;
  ros::Publisher _pubAxis1;
  ros::Publisher _pubAxis2;
  ros::Publisher _pubAxis3;

  ros::Subscriber _subJoy;
  ros::Subscriber _subDiagonstics;

  ros::ServiceClient _srv_robotic_arm_stand_by;
  ros::ServiceClient _srv_robotic_arm_active;

  ros::ServiceClient _srv_sw_drive_image;

  ros::Timer _loopTimer;
  ros::Timer _loopModeTimer;

  sensor_msgs::Joy _joy;

  std_msgs::String _drive_action;

  bool _joystick_rdy = false;
  bool _first_msg_rdy = false;
  double _rate;

  double _max_lin_vel;
  double _max_ang_vel;

  double _max_sh_vel;

  double _axis_factor;
  double _axis_offset;

  francor_msgs::SensorHeadCmd _sh_default;

  std::unique_ptr<francor::JoyMap> _joy_mapper;

  enum_modes _mode;

  bool _reverse_drive;

  bool _twist_enabled;

  std::string _front_cam_topic;
  std::string _back_cam_topic;

  const std::string DRIVE_ACTION_NONE = "NONE";
  const std::string DRIVE_ACTION_BOGIE_UP = "BOGIE_UP";
  const std::string DRIVE_ACTION_BOGIE_UP_DRIVE = "BOGIE_UP_DRIVE";
  const std::string DRIVE_ACTION_CLIMP = "CLIMP";
  const std::string DRIVE_ACTION_BOGIE_UP_BOOSTED = "BOGIE_UP_BOOSTED";
};

#endif /* FRANCORJOY2VEL_H_ */
