#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <memory>
#include <chrono>
#include <iostream>
using namespace std;
 
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_2"); //노드명
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub1_2", qos_profile); //토픽명
 
    RCLCPP_INFO(node->get_logger(), "실수 3개 입력");
    double x, y, z;
    cin >> x >> y >> z;
 
    geometry_msgs::msg::Vector3 message;
 
    while(rclcpp::ok()) {
        message.x = x;
        message.y = y;
        message.z = z;
        mypub->publish(message);
 
        rclcpp::sleep_for(chrono::nanoseconds(1s));
    }
 
    rclcpp::shutdown();
    return 0;
}
