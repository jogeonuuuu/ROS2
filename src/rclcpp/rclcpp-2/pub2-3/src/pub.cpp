#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono> // 사용자 정의 리터럴
#include <functional>
#include <iostream>
using namespace std::chrono_literals;
 
void callback(rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
    geometry_msgs::msg::Twist twist)
{
    RCLCPP_INFO(node->get_logger(), "Linear.x: %lf ,Angular.z: %lf", twist.linear.x, twist.angular.z);
    pub->publish(twist);
}
 
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mynode");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile);
 
    geometry_msgs::msg::Twist message;
    std::cout << "선속도(linear.x)와 각속도(angular.z)값을 작성하시오 >> ";
    std::cin >> message.linear.x >> message.angular.z;
 
    std::function<void()> fn = std::bind(callback, node, pub, message); //반환형을 auto로 하면 안됨.
                                                               //create_wall_timer 두 번째 매개변수의 조건 때문)
    auto timer = node->create_wall_timer(10ms, fn); //CallbackT자료형인 fn(반환형: void, 매개변수: X) 
    rclcpp::spin(node);
 
    rclcpp::shutdown();
    return 0;
}
