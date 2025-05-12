#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1; //새로운 함수(fn)의 첫번째 인자

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \ 
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host = 203.234.58.162 port = 9011 sync=false";

cv::VideoWriter writer;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg, cv::VideoWriter& output)
{
    //압축된 이미지를 cv::Mat 형식으로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::imshow("wsl", frame);
    cv::waitKey(1);
  
    writer << frame;
    output << frame;
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
 
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
 
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); //'D', 'I', 'V', 'X'
    double fps = 30;
    cv::VideoWriter outputVideo("camera1-2.mp4", fourcc, fps, cv::Size(640, 360));
 
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1, outputVideo);
 
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos, fn);
    rclcpp::spin(node);
 
    rclcpp::shutdown();
    return 0;
}
