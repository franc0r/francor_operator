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
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <francor_joy2vel/JoyMap.h>
#include <francor_joy2vel/JoyMapSc.h> //steam controller differential



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

  geometry_msgs::Twist getEmptyTwist() const
  {
    geometry_msgs::Twist empty;
    empty.linear.x = 0.0;
    empty.linear.y = 0.0;
    empty.angular.z = 0.0;
    return empty;
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
      _pubPosSensorHead.publish(_sh_default);
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
  }

  void btn_joystick_left_pressed()
  {
    ROS_INFO("Button JSL");
    //change mode
    if(_mode == DRIVE)
    {
      _mode = MANIPULATE_DIRECT;
    }
    else if(_mode == MANIPULATE_DIRECT)
    {
      _mode = MANIPULATE_INVERSE;
    }
    else if(_mode == MANIPULATE_INVERSE)
    {
      _mode = DRIVE;
    }
    else
    {
      //INVALID MODE -> go to DRIVE mode
      _mode = DRIVE;
    }
  }

  void btn_joystick_right_pressed()
  {
    ROS_INFO("Button JSR");
  }



private:    //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pubMode;
  ros::Publisher _pubTwist;
  ros::Publisher _pubSpeedSensorHead;
  ros::Publisher _pubPosSensorHead;
  ros::Publisher _pubRoboticArm;

  ros::Subscriber _subJoy;
  ros::Subscriber _subDiagonstics;

  ros::ServiceClient _srv_robotic_arm_stand_by;
  ros::ServiceClient _srv_robotic_arm_active;

  ros::Timer _loopTimer;
  ros::Timer _loopModeTimer;

  sensor_msgs::Joy _joy;

  bool _joystick_rdy = false;
  bool _first_msg_rdy = false;
  double _rate;

  double _max_lin_vel;
  double _max_ang_vel;

  double _max_sh_vel;

  francor_msgs::SensorHeadCmd _sh_default;

  std::unique_ptr<francor::JoyMap> _joy_mapper;

  enum_modes _mode;
};

#endif /* FRANCORJOY2VEL_H_ */
