
#include "FrancorOverlayHackNode.h"

FrancorOverlayHackNode::FrancorOverlayHackNode() :
  _it(_nh)
{
  //rosParam
  ros::NodeHandle privNh("~");
  
  double start_x;
  double start_y;
  double end_x;
  double end_y;


  privNh.param<double>( "start_x" ,    start_x,   100.0);
  privNh.param<double>( "start_y" ,    start_y,   100.0);
  privNh.param<double>( "end_x" ,    end_x,   200.0);
  privNh.param<double>( "end_y" ,    end_y,   200.0);

  _p_start = cv::Point2d(start_x, start_y);
  _p_end = cv::Point2d(end_x, end_y);


  //init publisher

  _sub_image = _it.subscribe("/camera/image", 1, &FrancorOverlayHackNode::sub_image_callback, this);
  _pub_image = _it.advertise("/driver/overlay", 1, true);
}

FrancorOverlayHackNode::~FrancorOverlayHackNode()
{
}

void FrancorOverlayHackNode::start(double duration)
{
  //create timer
  _loopTimer = _nh.createTimer(ros::Duration(duration), &FrancorOverlayHackNode::loop_callback, this);
  this->run();
}

void FrancorOverlayHackNode::run()
{
  ros::spin();
}

void FrancorOverlayHackNode::loop_callback(const ros::TimerEvent& e)
{
  //do loop stuff here!!!
  // ROS_INFO("beebob");
}

void FrancorOverlayHackNode::sub_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  // ROS_INFO("hans");
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat input = cv_ptr->image;

  //draw here
  cv::line(input, _p_start, _p_end, cv::Scalar(58,145,255), 2);
  

  std_msgs::Header hdr;
  hdr.frame_id = "cam";
  hdr.seq      = 1337;
  hdr.stamp    = ros::Time::now();

  //find encoding:
  std::string encoding;
  int t = input.type();
  // std::cout << "input.type(): " << input.type() << std::endl;
  switch (t) {
    case 0:
      encoding = "mono8";
      break;
    case 16:
      encoding = "bgr8";
      break;
    case 2:
      encoding = "mono16";
      break;
    default:
      ROS_ERROR("hans: cv::Mat type no supported... do nothing...");
      return;
  }
  cv_bridge::CvImage cv_img(hdr, encoding, input);
  // std::cout << "encoding: " << encoding << std::endl;

  _pub_image.publish(cv_img.toImageMsg());
}


// ------------- main ---------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "francor_overlay_hack_node");
  ros::NodeHandle nh("~");

  FrancorOverlayHackNode node;
  node.start();
}
