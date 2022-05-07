#include <memory>
#include <rclcpp/executors.hpp>

// #include "francor_joy2vel/Joy2VelSrvNode.hpp"
#include "francor_joy2vel/FrancorJoy2Vel.h"



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FrancorJoy2Vel>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}