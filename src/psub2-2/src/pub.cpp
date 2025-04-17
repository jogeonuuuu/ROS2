#include "psub2-2/pub.hpp"
Pub::Pub() : Node("mypub"), count_(0) //rclcpp::Node 클래스로부터 상속받은 파생클래스(this==node)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("mytopic", qos_profile);
    timer_ = this->create_wall_timer(1s, std::bind(&Pub::publish_msg, this));
    //멤버함수의 주소: &클래스명::멤버함수명
    //멤버함수는 this라는 매개변수를 가지기 때문에 포함해야 됨. (첫 번째 인자)
}
void Pub::publish_msg()
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = msg.y = msg.z = count_++;
    RCLCPP_INFO(this->get_logger(), "Published message: %lf, %lf, %lf", msg.x, msg.y, msg.z);
    pub_->publish(msg);
}