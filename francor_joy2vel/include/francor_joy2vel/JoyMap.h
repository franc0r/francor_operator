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

#include <functional>

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp> //todo prove if needed
// #include <std_msgs/Float64MultiArray.h>

// #include <francor_msgs/SensorHeadCmd.h>
#include "francor_msgs/msg/manipulator_cmd.hpp"

namespace francor{

namespace btn{
enum enum_JoyMapBtn{
  X = 0,
  Y,
  A,
  B,
  JS_L,
  JS_R,
  UP,
  DOWN,
  LEFT,
  RIGHT,
  TR_L,
  TR_R,
  SHARE,
  OPTIONS,
  NUM_BTN
};
} //namespace btn

struct JoyInput{
  bool init_ok = false;
  // double init_1 = 0.0;  ///< for init controller e.g. press both speed padles (e.g. needed for steamcontroller) just add the corresponding axis or button
  // double init_2 = 0.0;  ///< for init controller e.g. press both speed padles (e.g. needed for steamcontroller
  // double init_val = 0.0;  ///< value it neets to be for init rdy..

  double vel_lin_x = 0.0;
  double vel_lin_y = 0.0;   //for future use ... mecanum..
  double vel_ang    = 0.0;
  double vel_ang_up = 0.0;

  double vel_sh_pan  = 0.0;
  double vel_sh_tilt = 0.0;

  double vel_res = 0.0;

  bool vel_pressed = false;

  std::vector<bool> btns = std::vector<bool>(btn::NUM_BTN, false);
};




struct JoyMapButton_callbacks{
  
};

class JoyMap{
public:
  JoyMap(const double dead_zone_sh)
  {
    if(dead_zone_sh > 1.0)
    {
      _dead_zone_sh = 1.0;
    }
    _dead_zone_sh = std::abs(dead_zone_sh);
  }
  virtual ~JoyMap()
  { }

  inline geometry_msgs::msg::TwistStamped toTwistStamped(const double lin_scale = 1.0, const double ang_scale = 1.0, const bool reverse = false) const
  {
    geometry_msgs::msg::TwistStamped twist;
    // twist.header.stamp = ros::Time::now(); //todo
    twist.twist.angular.z = _input.vel_ang * ang_scale;
    twist.twist.linear.x  = _input.vel_lin_x * lin_scale * (reverse ? -1.0 : 1.0);
    twist.twist.linear.y  = _input.vel_lin_y * lin_scale;
    return twist;
  }
  inline geometry_msgs::msg::Twist toTwist(const double lin_scale = 1.0, const double ang_scale = 1.0, const bool reverse = false) const
  {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = _input.vel_ang * ang_scale;
    twist.linear.x  = _input.vel_lin_x * lin_scale * (reverse ? -1.0 : 1.0);
    twist.linear.y  = _input.vel_lin_y * lin_scale;
    return twist;
  }

  // inline francor_msgs::SensorHeadCmd toSensorHeadCmd(const double scale = 1.0, const bool reverse = false)
  // {
  //   francor_msgs::SensorHeadCmd cmd;
    
  //   // int16_t cmd_pan  = 500 + 1000 + std::round(1000.0 * _input.vel_sh_pan);
  //   // int16_t cmd_tilt = 500 + 1000 + std::round(1000.0 * _input.vel_sh_tilt);
  //   int16_t cmd_pan  = std::round(scale * _input.vel_sh_pan);
  //   int16_t cmd_tilt = std::round(scale * _input.vel_sh_tilt * (reverse ? -1.0 : 1.0));

  //   cmd.pan  = cmd_pan;
  //   cmd.tilt = cmd_tilt;
  //   return cmd;
  // }

  /**
   * @brief converts joy to sensorhead speed 
   * 
   * @param reverse first = pan, second = tilt
   * @return std::pair<double, double> 
   */
  inline std::pair<double, double> toSensorHeadSpeed(const bool reverse = false)
  {
    std::pair<double, double> speed;
    speed.first  = _input.vel_sh_pan;
    speed.second = _input.vel_sh_tilt * (reverse ? -1.0 : 1.0);
    return speed;
  }

  // inline std_msgs::Float64MultiArray toRoboicArmCmd()
  // {
  //   std_msgs::Float64MultiArray cmd;
  //   cmd.data.resize(6);
  //   cmd.data[0] = _input.vel_ang;
  //   cmd.data[1] = _input.vel_ang_up;
  //   cmd.data[2] = _input.vel_sh_tilt;
  //   cmd.data[3] = _input.vel_sh_pan;
  //   cmd.data[4] = _input.vel_lin_x;
  //   cmd.data[5] = _input.vel_res;

  //   return cmd;
  // }

  inline francor_msgs::msg::ManipulatorCmd toManipulatorCmd(const bool head_only = false)
  {
    //movement
    /* head pan tilt like SensorHeadCmd
     * head roll: x and b button
     * head gripper: close y open y&a
     * 
     * axis 1: ang_up
     * axis 2: ang
     */
    francor_msgs::msg::ManipulatorCmd cmd;

    if(head_only)
    {
      cmd.joint_0 = 0.0;
      cmd.joint_1 = 0.0;
      cmd.joint_2 = 0.0;
      cmd.joint_3 = 0.0;
    }
    else
    {
      cmd.joint_0      = _input.vel_ang;
      cmd.joint_1      = _input.vel_ang_up;
      cmd.joint_2      = _input.vel_lin_x;
      cmd.joint_3      = 0.0;
    }
    cmd.head_pan     = _input.vel_sh_pan * -1;
    cmd.head_tilt    = _input.vel_sh_tilt * -1;
    cmd.head_roll    = (_input.btns[btn::X] ? -0.2 : 0.0)  + (_input.btns[btn::B] ? 0.2 : 0.0);
    cmd.head_gripper = ((_input.btns[btn::Y]) && !_input.btns[btn::A] ? 0.3 : 0.0) + ((_input.btns[btn::Y] && _input.btns[btn::A]) ? -0.3 : 0.0);
    return cmd;
  }

  inline geometry_msgs::msg::Vector3 toManipulatorCmd_inverse()
  {
    geometry_msgs::msg::Vector3 cmd;

    cmd.x = _input.vel_lin_x;  
    cmd.y = _input.vel_ang;  //currently not used
    cmd.z = _input.vel_ang_up;

    return cmd;
  }

  void map(const sensor_msgs::msg::Joy& joy_msg)
  {
    _input = this->map_input(joy_msg);
  }

  JoyInput getJoyInput() const
  {
    return _input;
  }

  static inline double signum(const double x)
  {
    return (x > 0) - (x < 0);
  }

  void attach_callback(const btn::enum_JoyMapBtn btn, const std::function<void(void)>& fcn)
  {
    try{
      _callbacks.at(btn) = fcn;
    } catch(std::out_of_range& e)
    {
      // ROS_ERROR_STREAM("Unable to attach callback wront btn -> out of range. what?:"  << e.what());
      std::cout << "Unable to attach callback wront btn -> out of range. what?:"  << e.what() << std::endl;
      return;
    }
  }

  void triggerBtn_callbacks()
  {
    for(unsigned int i = 0; i < static_cast<unsigned int>(btn::NUM_BTN); i++)
    {
      //prove event
      if(!_btn_old[i] && _input.btns[i])
      {
        //call callback
        if(_callbacks[i])
          _callbacks[i]();
      }
    }
    //save old
    _btn_old = _input.btns;
  }

  //todo fcn to button
  //todo fcn to vel sensorhead..
  virtual void showInitMsg() const = 0;
protected:
  virtual JoyInput map_input(const sensor_msgs::msg::Joy& joy_msg) = 0;
  double _dead_zone_sh;

private:
  JoyInput _input;

  std::vector<std::function<void(void)>> _callbacks = std::vector<std::function<void(void)>>(btn::NUM_BTN);
  std::vector<bool> _btn_old = std::vector<bool>(btn::NUM_BTN, false);
};

} //namespace francor

#endif  //JOYMAP_H_