#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "dxl/dxl.hpp"
#include <memory>
#include <functional>
using namespace std::placeholders;

bool tf = false;

void mysub_callback(rclcpp::Node::SharedPtr node, Dxl& dxl, const std_msgs::msg::Int32::SharedPtr error)
{
    int lmotor, rmotor;

    if (Dxl::kbhit()) {
        if (dxl.getch() == 's') tf = !tf;
        if(!tf) lmotor=rmotor=0;
    }

    float gain = 1.5; //0.31
    lmotor = (100 - (gain * error->data));
    rmotor = -(100 + (gain * error->data));
    if(tf) dxl.setVelocity(lmotor, rmotor);

    RCLCPP_INFO(node->get_logger(), "error: %d", error->data);
    RCLCPP_INFO(node->get_logger(), "motor: %d, %d", lmotor, rmotor);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    Dxl dxl;
    auto node = std::make_shared<rclcpp::Node>("dxl_sub");
    if(!dxl.open())
    {
        RCLCPP_ERROR(node->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        return -1;
    }
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, dxl, _1);
    auto mysub = node->create_subscription<std_msgs::msg::Int32>("error",qos_profile,fn);
    rclcpp::spin(node);
    
    dxl.close();
    rclcpp::shutdown();
    return 0;
}
