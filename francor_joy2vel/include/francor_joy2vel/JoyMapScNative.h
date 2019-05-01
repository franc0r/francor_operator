/**
 * @file   JoyMapScNative.h
 * @author Michael Schmidpeter
 * @date   2018-03-12
 * @brief  todo
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */
#ifndef JOYMAPSCNATIVE_H_
#define JOYMAPSCNATIVE_H_




#include "JoyMap.h"

namespace francor{

class JoyMapScNative : public JoyMap{
public:
  // inherit from baseclass
  using JoyMap::JoyMap;
  virtual ~JoyMapScNative()
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
    double forward = joy_msg.axes[6];
    forward -= 1; //set to 0
    forward *= -1; //ivert to 0..1.5
    forward /= 1.5;

    forward = (forward > 1 ? 1 : forward);

    forward *= forward;

    //ROS_INFO("forward: %f", forward);

    double back    = joy_msg.axes[7];
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

    double rot_up     = joy_msg.axes[1];
    sign = rot_up < 0 ? -1 : 1;
    rot_up /= 0.85;
    rot_up = (std::abs(rot_up) > 1 ? 1 : rot_up);

    rot_up = std::abs(rot_up);
    //rot_up *= rot_up;
    rot_up *= sign;

    input.vel_lin_x = forward + back;
    input.vel_ang   = rot;
    input.vel_ang_up = rot_up * -1;

    //sensor head
    double sh_pan = joy_msg.axes[3];
    double sh_tilt = joy_msg.axes[4] * -1.0;
    if(sh_pan != 0.0 && sh_tilt != 0.0)
    {
      double sh_pan_sgn = this->signum(sh_pan);
      double sh_tilt_sgn = this->signum(sh_tilt);

      sh_pan = std::abs(sh_pan) - _dead_zone_sh;
      sh_tilt = std::abs(sh_tilt) - _dead_zone_sh;

      if(sh_pan < 0.0)
      {
        sh_pan = 0.0;
      }
      if(sh_tilt < 0.0)
      {
        sh_tilt = 0.0;
      }

      sh_pan /= (1 - _dead_zone_sh); //sclae to 1
      sh_pan *= sh_pan_sgn;
      sh_tilt /= (1 - _dead_zone_sh); //sclae to 1
      sh_tilt *= sh_tilt_sgn;

    }
    input.vel_sh_pan = sh_pan;
    input.vel_sh_tilt = sh_tilt;

    // input.vel_sh_pan = joy_msg.axes[0];
    // input.vel_sh_tilt = joy_msg.axes[1] * -1.0;


    // init values... 
    // input.init_1 = joy_msg.axes[5];
    // input.init_2 = joy_msg.axes[2];
    // input.init_val = -0.9;
    if(joy_msg.axes[6] < -0.99 && joy_msg.axes[7] < -0.99)
    {
      input.init_ok = true;
    }

    //todo buttons...

    
    input.btns[btn::X] = joy_msg.buttons[2];
    input.btns[btn::Y] = joy_msg.buttons[3];
    input.btns[btn::A] = joy_msg.buttons[0];
    input.btns[btn::B] = joy_msg.buttons[1];
    input.btns[btn::JS_L] = joy_msg.buttons[9];
    input.btns[btn::JS_R] = joy_msg.buttons[10];
    input.btns[btn::UP] = joy_msg.buttons[0];           //todo
    input.btns[btn::DOWN] = joy_msg.buttons[0];           //todo
    input.btns[btn::LEFT] = joy_msg.buttons[0];           //todo
    input.btns[btn::RIGHT] = joy_msg.buttons[0];            //todo
    input.btns[btn::TR_L] = joy_msg.buttons[4];
    input.btns[btn::TR_R] = joy_msg.buttons[5];


    return input;
  }

};

} //namespace francor

#endif  //JOYMAPSCNATIVE_H_