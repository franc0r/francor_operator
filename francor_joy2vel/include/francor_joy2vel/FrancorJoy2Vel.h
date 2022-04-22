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

#include <francor_msgs/msg/detail/manipulator_cmd__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <iostream>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_srvs/srv/detail/empty__struct.hpp>
#include <stdexcept>
#include <string>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
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

  bool executeEmptyService(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client)
  {
    //todo hacky spin_until_future_complete is bad here .... try furture_wait or something else maybe lambda  as callback and wait for it to finish

    try {
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto result_future =  client->async_send_request(request);
      if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
      {
        // RCLCPP_INFO(this->get_logger(), "Service executed successfully");
        return true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Service execution failed");
        return false;
      }
    } catch (std::runtime_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      return false;
    }
  }

  bool executeMuxSelectService(rclcpp::Client<topic_tools_interfaces::srv::MuxSelect>::SharedPtr client, std::string topic)
  {
    auto request = std::make_shared<topic_tools_interfaces::srv::MuxSelect::Request>();
    request->topic = topic;
    auto result_future =  client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      // RCLCPP_INFO(this->get_logger(), "Service executed successfully");
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Service execution failed");
      return false;
    }
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
    if(_reverse_drive)
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
    // ROS_INFO("Trigger left");
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
    // ROS_INFO("Button x");
    RCLCPP_INFO(this->get_logger(), "Button x");
    if(_mode == DRIVE)
    {
      //disable / enable Twist;
      if(_twist_enabled)
      {
        // ROS_INFO("Disable Twist");
        RCLCPP_INFO(this->get_logger(), "Disable Twist");
        _twist_enabled = false;
      }
      else
      {
        // ROS_INFO("Enable Twist");
        RCLCPP_INFO(this->get_logger(), "Enable Twist");
        _twist_enabled = true;
      }
    }
  }
  void btn_y_pressed()
  {
    // ROS_INFO("Button y");
    RCLCPP_INFO(this->get_logger(), "Button y");
  }

  void btn_a_pressed()
  {
    // ROS_INFO("Button a");
    RCLCPP_INFO(this->get_logger(), "Button a");
  }

  void btn_b_pressed()
  {
    // ROS_INFO("Button b");
    RCLCPP_INFO(this->get_logger(), "Button b");
  }

  void btn_joystick_left_pressed()
  {
    // ROS_INFO("Button JSL");
    RCLCPP_INFO(this->get_logger(), "Button JSL");
    //change mode
    if(_mode == DRIVE)
    {
      // ROS_INFO("Set Mode: MANIPULATE_DIRECT");
      RCLCPP_INFO(this->get_logger(), "Set Mode: MANIPULATE_DIRECT");
      //call service for axis mode
      // std_srvs::Empty srv;
      // 
      if(!this->executeEmptyService(_srv_set_manipulator_axis_mode))//_srv_set_manipulator_axis_mode.call(srv))
      {
        // ROS_ERROR("Unable to call _srv_set_manipulator_axis_mode ...");
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_set_manipulator_axis_mode ...");
      }
      _mode = MANIPULATE_DIRECT;
    }
    else if(_mode == MANIPULATE_DIRECT)
    {
      // ROS_INFO("Set Mode: MANIPULATE_INVERSE");
      RCLCPP_INFO(this->get_logger(), "Set Mode: MANIPULATE_INVERSE");
      //call service for inverse mode
      // std_srvs::Empty srv;
      if(!this->executeEmptyService(_srv_set_manipulator_inverse_mode))//!_srv_set_manipulator_inverse_mode.call(srv))
      {
        // ROS_ERROR("Unable to call _srv_set_manipulator_inverse_mode ...");
        RCLCPP_ERROR(this->get_logger(), "Unable to call _srv_set_manipulator_inverse_mode ...");
      }
      _mode = MANIPULATE_INVERSE;
    }
    else if(_mode == MANIPULATE_INVERSE)
    {
      // ROS_INFO("Set Mode: DRIVE");
      RCLCPP_INFO(this->get_logger(), "Set Mode: DRIVE");
      _mode = DRIVE;
    }
    else
    {
      //INVALID MODE -> go to DRIVE mode
      // ROS_INFO("Invalid Mode -> Set Mode: DRIVE");
      RCLCPP_INFO(this->get_logger(), "Invalid Mode -> Set Mode: DRIVE");
      _mode = DRIVE;
    }
  }

  void btn_joystick_right_pressed()
  {
    // ROS_INFO("Button JSR");
    RCLCPP_INFO(this->get_logger(), "Button JSR");

  }

  void btn_up_pressed()
  {
    // ROS_INFO("Button UP");
    RCLCPP_INFO(this->get_logger(), "Button UP");
    if(_mode == DRIVE)
    {
      //toggle reverse
      _reverse_drive = !_reverse_drive;
      //swich sensorhead to default
      // _pubPosSensorHead.publish(this->getDefaultSensorHead());
      std_msgs::msg::Float64 msg;
      msg.data = 0.0;
      _pubServoPanPos->publish(msg);
      _pubServoTiltPos->publish(this->getDefaultTiltSensorHead());
  
      if(_reverse_drive)
      {
        //swich driver cam topic
        // topic_tools_interfaces::srv::MuxSelect srv;
        // srv.request.topic = _back_cam_topic;
        if(!this->executeMuxSelectService(_srv_sw_drive_image, _back_cam_topic))//_srv_sw_drive_image.call(srv))
        {
          // ROS_WARN("Unable to call swich topic srv");
          RCLCPP_WARN(this->get_logger(), "Unable to call swich topic srv");
        }
      }
      else
      {
        // swich driver cam topic
        // topic_tools::MuxSelect srv;
        // srv.request.topic = _front_cam_topic;
        if(!this->executeMuxSelectService(_srv_sw_drive_image, _front_cam_topic))//_srv_sw_drive_image.call(srv))
        {
          // ROS_WARN("Unable to call swich topic srv");
          RCLCPP_WARN(this->get_logger(), "Unable to call swich topic srv");
        }
      }
    }
  }

  void btn_share_pressed()
  {
    // ROS_INFO("Button share");
    RCLCPP_INFO(this->get_logger(), "Button share");
    std_msgs::msg::Bool msg;
    msg.data = true;
    _pubAddVictim->publish(msg);
  }

private:    //dataelements
  // ros::NodeHandle _nh;

  // ros::Publisher _pubMode;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubMode;
  // ros::Publisher _pubTwist;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pubTwist;
  // ros::Publisher _pubTwistStamped;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _pubTwistStamped;
  // ros::Publisher _pubServoPanSpeed;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoPanSpeed;
  // ros::Publisher _pubServoTiltSpeed;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoTiltSpeed;
  // ros::Publisher _pubServoPanPos;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoPanPos;
 // ros::Publisher _pubServoTiltPos;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoTiltPos;
  // ros::Publisher _pubDriveAction;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pubDriveAction;
  // ros::Publisher _pubManipulatorAxisSpeed;
  rclcpp::Publisher<francor_msgs::msg::ManipulatorCmd>::SharedPtr _pubManipulatorAxisSpeed;
  // ros::Publisher _pubManipulaotrInverseSpeed;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _pubManipulaotrInverseSpeed;
  // ros::Publisher _pubAddVictim;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pubAddVictim;

  
  // ros::Subscriber _subJoy;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subJoy;
  // ros::Subscriber _subDiagonstics;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr _subDiagnostics;

  //todo
  // ros::ServiceClient _srv_robotic_arm_stand_by;
  // ros::ServiceClient _srv_robotic_arm_active;
  
  // ros::ServiceClient _srv_sw_drive_image;
  rclcpp::Client<topic_tools_interfaces::srv::MuxSelect>::SharedPtr _srv_sw_drive_image;
  // ros::ServiceClient _srv_set_manipulator_axis_mode;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _srv_set_manipulator_axis_mode;
  // ros::ServiceClient _srv_set_manipulator_inverse_mode;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _srv_set_manipulator_inverse_mode;

  // ros::Timer _loopTimer;
  rclcpp::TimerBase::SharedPtr _loopTimer;
  // ros::Timer _loopModeTimer;
  rclcpp::TimerBase::SharedPtr _loopModeTimer;

  sensor_msgs::msg::Joy::SharedPtr _joy;

  std_msgs::msg::String::SharedPtr _drive_action;

  bool _joystick_rdy = false;
  bool _first_msg_rdy = false;
  double _rate;

  double _max_lin_vel;
  double _max_ang_vel;

  // double _max_sh_vel;

  double _axis_factor;
  double _axis_offset;

  // francor_msgs::SensorHeadCmd _sh_default;
  double _sh_tilt_default;

  std::unique_ptr<francor::JoyMap> _joy_mapper;

  enum_modes _mode;

  bool _reverse_drive;

  bool _twist_enabled;

  std::string _front_cam_topic;
  std::string _back_cam_topic;
};

#endif /* FRANCORJOY2VEL_H_ */
