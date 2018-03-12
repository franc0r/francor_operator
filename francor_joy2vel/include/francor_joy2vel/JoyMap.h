/**
 * @file   JoyMap.h
 * @author Michael Schmidpeter
 * @date   2018-03-12
 * @brief  Joystick mapper..
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */
#ifndef JOYMAP_H_
#define JOYMAP_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

namespace francor{

struct JoyInput{
  bool init_ok = false;
  // double init_1 = 0.0;  ///< for init controller e.g. press both speed padles (e.g. needed for steamcontroller) just add the corresponding axis or button
  // double init_2 = 0.0;  ///< for init controller e.g. press both speed padles (e.g. needed for steamcontroller
  // double init_val = 0.0;  ///< value it neets to be for init rdy..

  double vel_lin_x = 0.0;
  double vel_lin_y = 0.0;   //for future use ... mecanum..
  double vel_ang = 0.0;

  //todo vel for sensorhead...

  bool btn_x = false;
  bool btn_y = false;
  bool btn_a = false;
  bool btn_b = false;
  bool btn_joystick_l = false;
  bool btn_joystick_r = false;
  bool btn_up = false;
  bool btn_down = false;
  bool btn_left = false;
  bool btn_right = false;
  bool btn_trigger_left = false;
  bool btn_trigger_right = false;
};

class JoyMap{
public:
  JoyMap()
  { }
  virtual ~JoyMap()
  { }

  geometry_msgs::TwistStamped toTwistStamped(const double lin_scale = 1.0, const double ang_scale = 1.0) const
  {
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.twist.angular.z = _input.vel_ang * ang_scale;
    twist.twist.linear.x  = _input.vel_lin_x * lin_scale;
    twist.twist.linear.y  = _input.vel_lin_y * lin_scale;
    return twist;
  }
  geometry_msgs::Twist toTwist(const double lin_scale = 1.0, const double ang_scale = 1.0) const
  {
    geometry_msgs::Twist twist;
    twist.angular.z = _input.vel_ang * ang_scale;
    twist.linear.x  = _input.vel_lin_x * lin_scale;
    twist.linear.y  = _input.vel_lin_y * lin_scale;
    return twist;
  }

  void map(const sensor_msgs::Joy& joy_msg)
  {
    _input = this->map_input(joy_msg);
  }

  JoyInput getJoyInput() const
  {
    return _input;
  }

  //todo fcn to button
  //todo fcn to vel sensorhead..
  virtual void showInitMsg() const = 0;
protected:
  virtual JoyInput map_input(const sensor_msgs::Joy& joy_msg) = 0;

private:
  JoyInput _input;
};

} //namespace francor

#endif  //JOYMAP_H_