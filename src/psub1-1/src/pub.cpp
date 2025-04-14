#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "rclcpp/time_source.hpp"
#include <memory>
#include <chrono>
#include <functional>
using namespace std::chrono_literals;
 
void callback(rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
    std_msgs::msg::String str)
{
    RCLCPP_INFO(node->get_logger(), "Publish: %s", str.data.c_str());
    pub->publish(str);
}
 
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<std_msgs::msg::String>("topic_pub1", qos_profile);
 
    std_msgs::msg::String message;
    message.data = "Hello world!";
 
    std::function<void()> fn = std::bind(callback, node, mypub, message);
    auto timer = node->create_wall_timer(100ms, fn);
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}