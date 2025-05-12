#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;
  
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    cv::Mat gray, binary;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
 
    cv::imshow("wsl", frame);
    cv::imshow("gray", gray);
    cv::imshow("binary", binary);
    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s, %d, %d", msg->format.c_str(), frame.rows, frame.cols);
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",qos,fn);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
