#ifndef _SUB_HPP_
#define _SUB_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1; //새로운 함수(fn)의 첫번째 인자
 
class Sub : public rclcpp::Node
{
private:
    cv::VideoWriter output;
    
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_; //SubscriptionT::SharedPtr
                                                                             //std::shared_ptr<SubscriptionT>
    void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
public:
    Sub();
};
#endif //_SUB_HPP_
