/**
 * @file   JoyMapSc.h
 * @author Michael Schmidpeter
 * @date   2018-03-12
 * @brief  todo
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */
#ifndef JOYMAPSC_H_
#define JOYMAPSC_H_

#include "JoyMap.h"

namespace francor{

class JoyMapSc : public JoyMap{
public:
  // inherit from baseclass
  using JoyMap::JoyMap;
  virtual ~JoyMapSc()
  { }

  virtual void showInitMsg() const
  {
    ROS_INFO("Initialize Joystick by pressing Max Forward and Max Backward simultaneously...");
  }
protected:
  virtual JoyInput map_input(const sensor_msgs::Joy& joy_msg)
  {
    JoyInput input;
    // -- velocity ------
    //5 forward    neutral:+1 max:-0.5
    //3 backward   neutral:+1 max:-0.5
    //2 angular    leftmax:1 , neutral:0, rightmax:-1
    
    // origin value is min->max 1 -> -0.5
    double forward = joy_msg.axes[5];
    forward -= 1; //set to 0
    forward *= -1; //ivert to 0..1.5
    forward /= 1.5;

    forward = (forward > 1 ? 1 : forward);

    forward *= forward;

    //ROS_INFO("forward: %f", forward);

    double back    = joy_msg.axes[2];
    back -= 1; //set to 0
    back *= -1; //ivert to 0..1.5
    back /= 1.5;

    back = (back > 1 ? 1 : back);

    back *= back; //scale parabola
    back *= -1;

    double rot     = joy_msg.axes[0];
    int sign = rot < 0 ? -1 : 1;
    rot /= 0.85;
    rot = (std::abs(rot) > 1 ? 1 : rot);

    rot = std::abs(rot);
    //rot *= rot;
    rot *= sign;

    input.vel_lin_x = forward + back;
    input.vel_ang   = rot;

    // init values...
    // input.init_1 = joy_msg.axes[5];
    // input.init_2 = joy_msg.axes[2];
    // input.init_val = -0.9;
    if(joy_msg.axes[5] < -0.99 && joy_msg.axes[2] < -0.99)
    {
      input.init_ok = true;
    }

    //todo buttons...
    return input;
  }

};

} //namespace francor

#endif  //JOYMAPSC_H_