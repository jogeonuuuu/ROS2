#include "camera1-3/pub.hpp"
 
std::string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";
 
Pub::Pub() : Node("campub_jetson") //rclcpp::Node 클래스로부터 상속받은 파생클래스(this==node)
{
    cap = cv::VideoCapture(src, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video!");
        rclcpp::shutdown();
    }
 
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);
    timer_ = this->create_wall_timer(40ms, std::bind(&Pub::publish_img, this));
    //멤버함수의 주소: &클래스명::멤버함수명
    //멤버함수는 this라는 매개변수를 가지기 때문에 포함해야 됨. (첫 번째 인자)
}
void Pub::publish_img()
{
    cap >> frame;
    if (frame.empty()) { 
        RCLCPP_ERROR(this->get_logger(), "frame empty");
        return;
    }
 
    std_msgs::msg::Header hdr; //메시지 헤더
    msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg(); //sensor_msgs::CompressedImagePtr
    //cv::Mat 객체(frame) -> CvImage 객체 -> sensor_msgs::msg::CompressedImage 객체(msg)
    pub_->publish(*msg); //msg는 객체 => std::shared_ptr<sensor_msgs::msg::CompressedImage>
}
