#include "lidar_sim/simulation.hpp"
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Simulation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}