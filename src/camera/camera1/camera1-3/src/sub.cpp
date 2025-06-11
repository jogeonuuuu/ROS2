#include "camera1-3/sub.hpp"
 
Sub::Sub() : Node("camsub_wsl"), output("camera1-4.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 360))
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile,
        std::bind(&Sub::mysub_callback, this, _1));
}
 
void Sub::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    //디코딩 : 압축된 이미지 -> cv::Mat 형식
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::imshow("wsl", frame);
    cv::waitKey(1);
 
    output << frame; //output 참조
}
