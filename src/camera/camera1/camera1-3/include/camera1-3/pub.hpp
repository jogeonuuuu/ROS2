#ifndef _PUB_HPP_
#define _PUB_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>
using namespace std::chrono_literals;
 
class Pub : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_; //TimerBase > WallTimer (업캐스팅)
 
    cv::VideoCapture cap;
    cv::Mat frame;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
 
    void publish_img();
public:
    Pub();
};
#endif //_PUB_HPP_
