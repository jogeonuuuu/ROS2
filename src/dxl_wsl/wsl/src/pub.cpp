#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
// #include "dxl_wsl/dxl.hpp"
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dxl_pub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("msg_value", qos_profile);
    geometry_msgs::msg::Vector3 vel;
    vel.x = vel.y = vel.z = 0;
    
    rclcpp::WallRate loop_rate(20.0); //20hz->50msec
    int vel1=0, vel2=0;
    int goal1=0, goal2=0;
    while(rclcpp::ok())
    {
        std::cout << "input >> ";
        char c='p';
        std::cin >> c;
        if(c!='p') {
            switch(c)
            {
            case ' ': vel.x = 0; vel.y = 0; break; //정지
            case 'w': vel.x = 50; vel.y = -50; break; //전진
            case 's': vel.x = -50; vel.y = 50; break; //후진
            case 'a': vel.x = -50; vel.y = -50; break; //좌회전
            case 'd': vel.x = 50; vel.y = 50; break; //우회전
            default : vel.x = 0; vel.y = 0; break;
            }
        }

        RCLCPP_INFO(node->get_logger(), "Publish: %lf,%lf", vel.x, vel.y);
        mypub->publish(vel);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
