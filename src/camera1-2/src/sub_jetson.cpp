#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

//Format: BGRx, GRAY8
std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host = 203.234.58.162 port = 9011 sync=false";

cv::VideoWriter writer;
        
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg, cv::VideoWriter& output) //, cv::VideoWriter& writer
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    cv::Mat gray, binary, binary_bgr;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  
    writer << binary;
    output << frame;
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("camsub");
    //cv::VideoWriter writer;
    writer.open(dst, 0, (double)30, cv::Size(640, 360), false);
    if(!writer.isOpened()) { RCLCPP_ERROR(node->get_logger(), "Writer open failed!"); rclcpp::shutdown(); return -1; }

    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); //'D', 'I', 'V', 'X' / 'm', 'p', '4', 'v' / 'M', 'J', 'P', 'G'
    double fps = 30;
    cv::VideoWriter outputVideo("camera1-2.avi", fourcc, fps, cv::Size(640, 360)); //mp4, avi

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1, outputVideo); //, writer

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, fn);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}