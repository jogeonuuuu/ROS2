#ifndef _FINAL_HPP_
#define _FINAL_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
// #include "final/dxl.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono>
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*(M_PI/180.))
#define MET2PXL(x) ((x)*50.) //500(pixel)이 10m로 가정했을 때 (가변적: 1미터당 50pixel로 변경)
#define AREA 100
 
using namespace std::chrono_literals;
using std::placeholders::_1;
 
class Final : public rclcpp::Node 
{
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;
    static cv::Mat lidar_img;
    static cv::VideoWriter output;
 
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
    static std_msgs::msg::Int32 error;
 
    //static double left_min_distance, right_min_distance;
 
    static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan);
    void pub_callback();
    static void error_calc(cv::Mat &lidar_img);
public:
    Final();
};

#endif //_FINAL_HPP_