#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono> // 사용자 정의 리터럴
#include <functional>
using namespace std::chrono_literals;
 
void callback(rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub)
    //std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> pub과 동일한 문장
{
    static int count = 0;
    std_msgs::msg::Int32 message;
    message.data = count++;
    RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data);
    pub->publish(message);
}
 
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mynode");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<std_msgs::msg::Int32>("mytopic", qos_profile);
 
    std::function<void()> fn = std::bind(callback, node, pub); //반환형을 auto로 하면 안됨.
                                                               //create_wall_timer 두 번째 매개변수의 조건 때문)
    auto timer = node->create_wall_timer(100ms, fn); //CallbackT자료형인 fn(반환형: void, 매개변수: X) 
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
