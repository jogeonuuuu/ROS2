#ifndef _WSL_HPP_
#define _WSL_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
//#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <thread>
#include <chrono>
#include <string>
 
using namespace std::chrono_literals;
using std::placeholders::_1;
 
class Info {
private:
    int index;
    double distance;
public:
    Info(int index, double distance) {
        this->index = index;
        this->distance = distance;
    }
    int get_index(void) {
        return this->index;
    }
    double get_distance() {
        return this->distance;
    }
};
 
class wsl : public rclcpp::Node 
{
private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
    cv::Mat original; //640 x 360
    cv::Mat toUse;    //640 x 90
    cv::VideoWriter original_video;
    cv::VideoWriter toUse_video;
    cv::Mat labels, stats, centroids; //stats : 각 레이블의 정보(x, y, width, hight)
                                      //centroids : 각 레이블의 무게중심
    cv::Point2d past_point, present_point;
    double distance; //중점으로부터 가장 가까운 객체와의 거리
 
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Int32 error;
 
    void img_show(const sensor_msgs::msg::CompressedImage::SharedPtr msg); //img sub
    void pub_callback(); //error pub
public:
    wsl();
};
 
#endif //_WSL_HPP_