#ifndef FRANCOROVERLAYHACKNODE_H_
#define FRANCOROVERLAYHACKNODE_H_



#include <ros/ros.h>
#include <std_msgs/Bool.h>
//dyn reconfig
// #include <dynamic_reconfigure/server.h>
//#include <FrancorOverlayHackNode/FrancorOverlayHackNodeConfig.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class FrancorOverlayHackNode
{

public:
  FrancorOverlayHackNode();
  virtual ~FrancorOverlayHackNode();

  /**
     *
     * @brief
     *
     * @return  void
     */
  void start(double duration = 1.0);

private: //functions
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

  void sub_image_callback(const sensor_msgs::ImageConstPtr& msg);

  //void subCallback(const ROS_PACK::MESSAGE& msg);

  //void dynreconfig_callback(FrancorOverlayHackNode::FrancorOverlayHackNodeConfig &config, uint32_t level);
private: //dataelements
  ros::NodeHandle _nh;

  image_transport::ImageTransport _it;
  image_transport::Subscriber _sub_image;
  image_transport::Publisher  _pub_image;


  cv::Point2d _p_start;
  cv::Point2d _p_end;

  // dynamic_reconfigure::Server<rona_frontier::ExplorationConfig> _drServer;


  ros::Timer _loopTimer;
};
#endif  //FRANCOROVERLAYHACKNODE_H_
