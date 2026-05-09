#include "hexapod_tripod_gait/tripod_gait_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexapod_tripod_gait::TripodGaitNode>());
  rclcpp::shutdown();
  return 0;
}
