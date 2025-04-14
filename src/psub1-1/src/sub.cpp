#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <functional>
using std::placeholders::_1; //unbind (fn함수의 첫 번째 인자)
void mysub_callback(rclcpp::Node::SharedPtr node, //std::shared_ptr<rclcpp::Node> node
    const std_msgs::msg::String::SharedPtr msg)   //std::shared_ptr<std_msgs::msg::String> msg
{
    RCLCPP_INFO(node->get_logger(), "Received message: %s", msg->data.c_str());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_sub1");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
 
    //토픽 메시지를 수신했을 때 호출되는 사용자 정의 콜백함수
    std::function<void(const std_msgs::msg::String::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1); //create_subscription함수의 세 번째 매개변수 조건
                                              //반환형: void, 매개변수: MessageT
    auto mysub = node->create_subscription<std_msgs::msg::String>("topic_pub1", qos_profile, fn);
    rclcpp::spin(node);
 
    rclcpp::shutdown();
    return 0;
}