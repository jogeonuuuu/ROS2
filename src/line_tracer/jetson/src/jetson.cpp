#include "line_tracer/jetson.hpp"
 
void jetson::img_pub()
{
    cap >> img;
    if (img.empty()) { 
        RCLCPP_WARN(this->get_logger(), "Empty frame captured!");
        return;
    }
 
    msg = cv_bridge::CvImage(hdr, "bgr8", img).toCompressedImageMsg();
    //cv::Mat 객체(frame) -> CvImage 객체 -> sensor_msgs::msg::CompressedImage 객체(msg)
    pub->publish(*msg);
}
 
void jetson::sub_callback(const std_msgs::msg::Int32::SharedPtr error)
//std::function<void(MessageT)> 형의 함수만 전달가능
{
    int lmotor, rmotor;
    if (Dxl::kbhit()) {
        if (dxl.getch() == 's') tf = !tf;
        if(!tf) lmotor=rmotor=0;
    }
 
    lmotor = (200 - (0.61 * error->data));
    rmotor = -(200 + (0.61 * error->data));
    if(tf) dxl.setVelocity(lmotor, rmotor);
    
    RCLCPP_INFO(this->get_logger(), "error: %d", error->data);
    RCLCPP_INFO(this->get_logger(), "motor: %d, %d", lmotor, rmotor);
}
 
jetson::jetson() : Node("jetson")
{
    if (!dxl.open()) {
        RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
    }
    cap = cv::VideoCapture("/home/jetson/ros2_ws/src/5_lt_cw_100rpm_out.mp4");
    pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    timer_ = this->create_wall_timer(30ms, std::bind(&jetson::img_pub, this));
 
    sub = this->create_subscription<std_msgs::msg::Int32>("error",
        rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&jetson::sub_callback, this, _1));
}
jetson::~jetson()
{
    dxl.close();
}