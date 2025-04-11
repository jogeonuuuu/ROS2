#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>
#include <iostream>
using namespace std;
 
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_3"); //노드명
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile); //토픽명, 
 
    geometry_msgs::msg::Twist message;
 
    char ch;
 
    while(rclcpp::ok()) {
        cin >> ch;
        switch(ch) {
        case 'f': //전진
            message.linear.x = 1.0; 
            message.angular.z = 0.0;
            break;
        case 'b': //후진
            message.linear.x = -1.0; 
            message.angular.z = 0.0;
            break;
        case 'l': //좌회전
            message.linear.x = 0.0; 
            message.angular.z = 1.0;
            break;
        case 'r': //우회전
            message.linear.x = 0.0; 
            message.angular.z = -1.0;
            break;
        default:
            break;
        }
 
        RCLCPP_INFO(node->get_logger(), "Linear.x: %d ,Angular.z: %d", message.linear.x, message.angular.z);
        mypub->publish(message);
 
        rclcpp::sleep_for(chrono::nanoseconds(1s));
    }
 
    rclcpp::shutdown();
    return 0;
}
