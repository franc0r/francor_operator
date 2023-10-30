/**
 * @file   JoyMapPs4.h
 * @author Michael Schmidpeter
 * @date   2018-03-12
 * @brief  todo
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */
#ifndef JOYMAPPS5_H_
#define JOYMAPPS5_H_




#include "JoyMap.h"

namespace francor{

class JoyMapPs5 : public JoyMap{
public:
  // inherit from baseclass
  using JoyMap::JoyMap;
  virtual ~JoyMapPs5()
  { }

  virtual void showInitMsg() const
  {
    // ROS_INFO("Initialize Joystick by pressing Max Forward and Max Backward simultaneously...");
    std::cout << "Initialize Joystick by pressing Max Forward and Max Backward simultaneously..." << std::endl;
  }
protected:
  virtual JoyInput map_input(const sensor_msgs::msg::Joy& joy_msg)
  {

    const double lin_fac_1 = 0.3;
    const double lin_fac_2 = 0.7;

    const double rot_fac_1 = 0.5;
    const double rot_fac_2 = 0.5;

    JoyInput input;
    // -- velocity ------
    //5 forward    neutral:+1 max:-0.5
    //2 backward   neutral:+1 max:-0.5
    //0 angular    leftmax:1 , neutral:0, rightmax:-1
    
    // origin value is min->max 1 -> -0.5
    double forward = joy_msg.axes[5];
    forward -= 1; //set to 0
    forward *= -1; //ivert to 0..1.5
    forward /= 2;

    forward = (forward > 1 ? 1 : forward);

    forward *= lin_fac_1 * forward + lin_fac_2 * forward * forward;

    //ROS_INFO("forward: %f", forward);

    double back    = joy_msg.axes[2];
    back -= 1; //set to 0
    back *= -1; //ivert to 0..1.5
    back /= 2;

    back = (back > 1 ? 1 : back);

    back *= lin_fac_1 * back + lin_fac_2 * back * back;

    back *= -1;

    double rot     = joy_msg.axes[0] * -1;
    int sign = rot < 0 ? -1 : 1;
    rot /= 0.85;
    rot = (std::abs(rot) > 1 ? 1 : rot);

    rot = std::abs(rot);
    rot *= rot_fac_1 * rot + rot_fac_2 * rot * rot;

    rot *= sign * -1;

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
    double sh_tilt = joy_msg.axes[4];
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
      sh_pan *= sh_pan; //add parabola
      sh_pan *= sh_pan_sgn;
      sh_tilt /= (1 - _dead_zone_sh); //sclae to 1
      sh_tilt *= sh_tilt; //ad parabola
      sh_tilt *= sh_tilt_sgn;

    }
    input.vel_sh_pan = sh_pan;
    input.vel_sh_tilt = sh_tilt;

    //vel res
    input.vel_res = joy_msg.buttons[0] + joy_msg.buttons[1] * -1;

    // input.vel_sh_pan = joy_msg.axes[0];
    // input.vel_sh_tilt = joy_msg.axes[1] * -1.0;


    // init values... 
    // input.init_1 = joy_msg.axes[5];
    // input.init_2 = joy_msg.axes[2];
    // input.init_val = -0.9;
    if(joy_msg.axes[5] < -0.99 && joy_msg.axes[2] < -0.99)
    {
      // std::cout << "INIT OK" << std::endl;
      input.init_ok = true;
    }

    if(joy_msg.axes[5] != 1.0 || joy_msg.axes[2] != 1.0)
    {
      input.vel_pressed = true;
    }

    input.btns[btn::X] = joy_msg.buttons[2];
    input.btns[btn::Y] = joy_msg.buttons[3];
    input.btns[btn::A] = joy_msg.buttons[0];
    input.btns[btn::B] = joy_msg.buttons[1];
    input.btns[btn::JS_L] = joy_msg.buttons[9];
    input.btns[btn::JS_R] = joy_msg.buttons[10];
    input.btns[btn::UP]    = joy_msg.axes[7] == 1.0;
    input.btns[btn::DOWN]  = joy_msg.axes[7] == -1.0;
    input.btns[btn::LEFT]  = joy_msg.axes[6] == 1.0;
    input.btns[btn::RIGHT] = joy_msg.axes[6] == -1.0;

    input.btns[btn::TR_L] = joy_msg.buttons[4];
    input.btns[btn::TR_R] = joy_msg.buttons[5];
    input.btns[btn::SHARE] = joy_msg.buttons[8];
    input.btns[btn::OPTIONS] = joy_msg.buttons[9];

    return input;
  }

};

} //namespace francor

#endif  //JOYMAPPS4_H_