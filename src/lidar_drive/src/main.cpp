#include "lidar_drive/lidar_drive.hpp"
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Final>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}