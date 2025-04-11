#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <memory>
#include <chrono> // 사용자 정의 리터럴
#include <functional>
#include <iostream>
using namespace std::chrono_literals;
 
void callback(rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub)
    //std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Vector3>> pub과 동일한 문장
{
    std::cout << "실수 3개 입력 >> ";
    double x, y, z;
    std::cin >> x >> y >> z;
 
    geometry_msgs::msg::Vector3 message;
    message.x = x;
    message.y = y;
    message.z = z;
 
    pub->publish(message);
}
 
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mynode");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("mytopic", qos_profile);
 
    std::function<void()> fn = std::bind(callback, node, pub); //반환형을 auto로 하면 안됨.
                                                               //create_wall_timer 두 번째 매개변수의 조건 때문)
    auto timer = node->create_wall_timer(3s, fn); //CallbackT자료형인 fn(반환형: void, 매개변수: X) 
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
