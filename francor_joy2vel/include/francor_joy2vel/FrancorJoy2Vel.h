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

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp> //todo prove if needed
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <topic_tools_interfaces/srv/mux_select.hpp> //// #include <topic_tools/msg/ MuxSelect.h>



#include "francor_joy2vel/JoyMap.h"
#include "francor_joy2vel/JoyMapSc.h" //steam controller differential
#include "francor_joy2vel/JoyMapPs4.h"



#include "francor_msgs/msg/manipulator_cmd.hpp"



class FrancorJoy2Vel : public rclcpp::Node
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
  void init();

private:    //functions

  /**
   *
   * @brief this function containts the main working loop
   *
   * @param[in,out]  void
   *
   * @return 		   void
   */
  // void run();

  void loop_callback();//const ros::TimerEvent& e);

  void loop_mode_callback();//const ros::TimerEvent& e);

  void subJoy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void subDiagnostic_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg); //for detecting joystic timeout...


  bool executeEmptySrv(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client)
  {
    using namespace std::chrono_literals;
    if(!client->wait_for_service(100ms))
    {
      // RCLCPP_ERROR(this->get_logger(), "Service not available, aborting");
      return false;
    }

    //fucking hack!!!!
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();

    using srv_res_future = rclcpp::Client<std_srvs::srv::Empty>::SharedFutureWithRequest;
    auto response_received_callback =
      [logger = this->get_logger()](srv_res_future future) {
        (void)future; //unused
        // auto request_response_pair = future.get();
        RCLCPP_INFO(logger, "Result of srv good!");
      };

    auto result = client->async_send_request(req, std::move(response_received_callback));
    return true;
  }

  bool executeSetBoolSrv(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value)
  {
    using namespace std::chrono_literals;
    if(!client->wait_for_service(100ms))
    {
      // RCLCPP_ERROR(this->get_logger(), "Service not available, aborting");
      return false;
    }

    //fucking hack!!!!
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = value;

    using srv_res_future = rclcpp::Client<std_srvs::srv::SetBool>::SharedFutureWithRequest;
    auto response_received_callback =
      [logger = this->get_logger()](srv_res_future future) {
        // (void)future; //unused
        auto request_response_pair = future.get();

        RCLCPP_INFO(logger, "Result of srv good! -> request: %s, res -> succ: %s str: %s", 
          (request_response_pair.first->data ? "true" : "false"), 
          (request_response_pair.second->success ? "true" : "false"), 
          request_response_pair.second->message.c_str());
      };

    auto result = client->async_send_request(req, std::move(response_received_callback));
    return true;
  }

  bool executeMuxSelectSrv(rclcpp::Client<topic_tools_interfaces::srv::MuxSelect>::SharedPtr client, std::string topic)
  {
    using namespace std::chrono_literals;
    if(!client->wait_for_service(100ms))
    {
      // RCLCPP_ERROR(this->get_logger(), "Service not available, aborting");
      return false;
    }
    //fucking hack!!!!
    auto req = std::make_shared<topic_tools_interfaces::srv::MuxSelect::Request>();
    req->topic = topic;

    using srv_res_future = rclcpp::Client<topic_tools_interfaces::srv::MuxSelect>::SharedFutureWithRequest;
    auto response_received_callback =
      [logger = this->get_logger()](srv_res_future future) {
        auto request_response_pair = future.get();
        // request_response_pair.second->prev_topic
        RCLCPP_INFO(logger, "Result of srv good! -> switched from  %s  to %s", request_response_pair.second->prev_topic.c_str(), request_response_pair.first->topic.c_str() );
      };

    auto result = client->async_send_request(req, std::move(response_received_callback));
    return true;
  }
  geometry_msgs::msg::TwistStamped getEmptyTwist() const
  {
    geometry_msgs::msg::TwistStamped empty;
    // empty.header.stamp = ros::Time::now(); //todo
    empty.twist.linear.x = 0.0;
    empty.twist.linear.y = 0.0;
    empty.twist.angular.z = 0.0;
    return empty;
  }

  //old sh
  // francor_msgs::SensorHeadCmd getDefaultSensorHead()
  std_msgs::msg::Float64 getDefaultTiltSensorHead()
  {
    // francor_msgs::SensorHeadCmd cmd = _sh_default;
    std_msgs::msg::Float64 cmd;
    cmd.data = _sh_tilt_default;
    if(!_reverse_drive)
    {
      cmd.data *= -1;
    }
    return cmd;
  }


  // std_msgs::String toDriveAction(const std::string& drive_action)
  // {
  //   std_msgs::String msg;
  //   msg.data = drive_action;
  //   return msg;
  // }

  void btn_trigger_left_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Trigger left");
    if(_mode == DRIVE)
    {
      std_msgs::msg::Float64 msg;
      msg.data = 0.0;
      _pubServoPanPos->publish(msg);
      _pubServoTiltPos->publish(getDefaultTiltSensorHead());
    }
    else if(_mode == MANIPULATE_DIRECT || _mode == MANIPULATE_INVERSE)
    {
      // std_srvs::Empty srv;
      //todo
      // if(_srv_robotic_arm_active.call(srv))
      // {
      //   ROS_INFO("Set arm active");
      // }
      // else
      // {
      //   ROS_WARN("Unable to call set arm active Service");
      // }
    }
  }
  void btn_trigger_right_pressed()
  {
    // ROS_INFO("Trigger right");
    RCLCPP_INFO(this->get_logger(), "Trigger right");
    if(_mode == DRIVE)
    {
      //old sh
      // _pubPosSensorHead.publish(this->getDefaultSensorHead());
      //new sh
      std_msgs::msg::Float64 msg;
      msg.data = 0.0;
      _pubServoPanPos->publish(msg);
      _pubServoTiltPos->publish(this->getDefaultTiltSensorHead());

    }
    else if(_mode == MANIPULATE_DIRECT || _mode == MANIPULATE_INVERSE)
    {
      //todo
      // std_srvs::Empty srv;
      // if(_srv_robotic_arm_stand_by.call(srv))
      // {
      //   ROS_INFO("Set arm stand by");
      // }
      // else
      // {
      //   ROS_WARN("Unable to call set arm stand by Service");
      // }
    }
  }
  void btn_x_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button x");
    if(_mode == DRIVE)
    {
      //disable / enable Twist;
      if(_twist_enabled)
      {
        RCLCPP_INFO(this->get_logger(), "Disable Twist");
        _twist_enabled = false;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Enable Twist");
        _twist_enabled = true;
      }
    }
  }
  void btn_y_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button y");
  }

  void btn_a_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button a");
  }

  void btn_b_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button b");
  }

  void btn_joystick_left_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button JSL");
    //change mode
    if(_mode == DRIVE)
    {
      RCLCPP_INFO(this->get_logger(), "Set Mode: MANIPULATE_DIRECT");
      //call service for axis mode
      if(!this->executeEmptySrv(_srv_manipulator_axis_mode))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_set_manipulator_axis_mode ...");
      }
      _mode = MANIPULATE_DIRECT;
    }
    else if(_mode == MANIPULATE_DIRECT)
    {
      RCLCPP_INFO(this->get_logger(), "Set Mode: MANIPULATE_INVERSE");
      //call service for inverse mode
      if(!this->executeEmptySrv(_srv_manipulator_inverse_mode))  
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_set_manipulator_inverse_mode ...");
      }
      _mode = MANIPULATE_INVERSE;
    }
    else if(_mode == MANIPULATE_INVERSE)
    {
      RCLCPP_INFO(this->get_logger(), "Set Mode: DRIVE");
      _mode = DRIVE;
    }
    else
    {
      //INVALID MODE -> go to DRIVE mode
      RCLCPP_INFO(this->get_logger(), "Invalid Mode -> Set Mode: DRIVE");
      _mode = DRIVE;
    }
  }

  void btn_joystick_right_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button JSR");

  }

  void btn_up_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button UP");
    if(_mode == DRIVE)
    {
      //toggle reverse
      _reverse_drive = !_reverse_drive;
      //swich sensorhead to default
      std_msgs::msg::Float64 msg;
      msg.data = 0.0;
      _pubServoPanPos->publish(msg);
      _pubServoTiltPos->publish(this->getDefaultTiltSensorHead());
  
      if(_reverse_drive)
      {
        //swich driver cam topic
        if(!this->executeMuxSelectSrv(_srv_sw_drive_image, _back_cam_topic))
        {
          RCLCPP_WARN(this->get_logger(), "Unable to call swich topic srv");
        }
      }
      else
      {
        // swich driver cam topic
        if(!this->executeMuxSelectSrv(_srv_sw_drive_image, _front_cam_topic))
        {
          RCLCPP_WARN(this->get_logger(), "Unable to call swich topic srv");
        }
      }
    }
  }

  void btn_right_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button RIGHT pressed");
    if(_mode == DRIVE)
    {
      //enable drives
      RCLCPP_INFO(this->get_logger(), "Enable drives");
      if(!this->executeSetBoolSrv(_srv_enable_drives, true))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_enable_drives ...");
      }
    }
    else if(_mode == MANIPULATE_DIRECT || _mode == MANIPULATE_INVERSE)
    {
      RCLCPP_INFO(this->get_logger(), "Set arm active");
      if(!this->executeEmptySrv(_srv_manipulator_set_active))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_manipulator_set_active ...");
      }
    }
  }

  void btn_left_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button LEFT pressed");
  }

  void btn_down_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button DOWN pressed");
    if(_mode == DRIVE)
    {
      //disable drives
      RCLCPP_INFO(this->get_logger(), "Disable drives");
      if(!this->executeSetBoolSrv(_srv_enable_drives, false))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_enable_drives ...");
      }
    }
    else if(_mode == MANIPULATE_DIRECT || _mode == MANIPULATE_INVERSE)
    {
      RCLCPP_INFO(this->get_logger(), "Set arm stand by");
      if(!this->executeEmptySrv(_srv_manipulator_set_stand_by))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_manipulator_stand_by ...");
      }
    }

  }

  void btn_share_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button share -> Send Victim");
    std_msgs::msg::Bool msg;
    msg.data = true;
    _pubAddVictim->publish(msg);
  }

  void btn_options_pressed()
  {
    RCLCPP_INFO(this->get_logger(), "Button options");
    if(_mode == MANIPULATE_DIRECT || _mode == MANIPULATE_INVERSE)
    {
      RCLCPP_INFO(this->get_logger(), "Set arm init_enable");
      if(!this->executeEmptySrv(_srv_manipulator_init_enable))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_manipulator_init_enable ...");
      }
    }
  }

private:    //dataelements
  // ros::NodeHandle _nh;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubMode;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pubTwist;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _pubTwistStamped;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoPanSpeed;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoTiltSpeed;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoPanPos;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoTiltPos;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubDriveAction;
  rclcpp::Publisher<francor_msgs::msg::ManipulatorCmd>::SharedPtr _pubManipulatorAxisSpeed;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _pubManipulaotrInverseSpeed;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pubAddVictim;

  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subJoy;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr _subDiagnostics;


  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _srv_manipulator_axis_mode;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _srv_manipulator_inverse_mode;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _srv_manipulator_set_active;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _srv_manipulator_set_stand_by;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _srv_manipulator_init_enable;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _srv_enable_drives;
  rclcpp::Client<topic_tools_interfaces::srv::MuxSelect>::SharedPtr _srv_sw_drive_image;


  rclcpp::TimerBase::SharedPtr _loopTimer;
  rclcpp::TimerBase::SharedPtr _loopModeTimer;
  //timer clear servcalls
  // rclcpp::TimerBase::SharedPtr _timerCleanSrvs;

  sensor_msgs::msg::Joy::SharedPtr _joy;

  std_msgs::msg::String::SharedPtr _drive_action;

  bool _joystick_rdy = false;
  bool _first_msg_rdy = false;
  double _rate;

  double _max_lin_vel;
  double _max_ang_vel;

  double _axis_factor;
  double _axis_offset;

  double _sh_tilt_default;

  std::unique_ptr<francor::JoyMap> _joy_mapper;

  enum_modes _mode;

  bool _reverse_drive;

  bool _twist_enabled;

  std::string _front_cam_topic;
  std::string _back_cam_topic;
};

#endif /* FRANCORJOY2VEL_H_ */
