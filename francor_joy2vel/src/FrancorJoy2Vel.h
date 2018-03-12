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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <francor_joy2vel/JoyMap.h>
#include <francor_joy2vel/JoyMapSc.h> //steam controller differential

class FrancorJoy2Vel
{

public:
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

  void subJoy_callback(const sensor_msgs::Joy& msg);
  void subDiagnostic_callback(const diagnostic_msgs::DiagnosticArray& msg); //for detecting joystic timeout...

private:    //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pubTwist;
  ros::Subscriber _subJoy;
  ros::Subscriber _subDiagonstics;

  ros::Timer _loopTimer;

  sensor_msgs::Joy _joy;

  bool _joystick_rdy = false;
  bool _first_msg_rdy = false;
  double _rate;

  double _max_lin_vel;
  double _max_ang_vel;

  std::unique_ptr<francor::JoyMap> _joy_mapper;
};

#endif /* FRANCORJOY2VEL_H_ */
