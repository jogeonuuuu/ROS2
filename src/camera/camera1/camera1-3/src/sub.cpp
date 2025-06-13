#include "camera1-3/sub.hpp"

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host = 203.234.xx.xxx port = xxxx sync=false";

cv::VideoWriter writer;
writer.open(dst, 0, (double)30, cv::Size(640, 360), false);

void Sub::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    //디코딩 : 압축된 이미지 -> cv::Mat 형식
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::imshow("wsl", frame);
    cv::waitKey(1);

    writer << frame;
    output << frame; //output 참조
}

Sub::Sub() : Node("camsub"), output("camera1-3.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 360))
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile,
        std::bind(&Sub::mysub_callback, this, _1));
}
