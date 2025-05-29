#include "line_tracer/wsl.hpp"
 
bool compare_info(Info &a, Info &b) {
    return a.get_distance() < b.get_distance();
}
 
void wsl::img_show(const sensor_msgs::msg::CompressedImage::SharedPtr msg) //img sub
{
    original = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR); //원본 이미지
    toUse = original(cv::Rect(0, original.rows*3/4, original.cols, original.rows/4)); //(1) 영상의 아랫부분 1/4
    cv::cvtColor(toUse, toUse, cv::COLOR_BGR2GRAY); //(2) 그레이스케일영상으로 변환
    toUse += 100 - cv::mean(toUse)[0]; //(3) 영상의 평균밝기를 원하는 값으로 조정
    cv::threshold(toUse, toUse, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); //(4) 이진화
    int lable_cnt =  cv::connectedComponentsWithStats(toUse, labels, stats, centroids); //(5) 레이블링
    cv::cvtColor(toUse, toUse, cv::COLOR_GRAY2BGR);
 
    std::vector<Info> v;
    for(int i = 1; i < lable_cnt; i++) { //'0'은 배경
        double *p = centroids.ptr<double>(i); //centroids 행렬의 깊이: CV_64F(double)
        int *q = stats.ptr<int>(i);
 
        if(q[4] > 100) { //면적이 100보다 큰 경우에만 
            distance = sqrt(pow((present_point.x - p[0]), 2) + pow((present_point.y - p[1]), 2));
            v.push_back(Info(i, distance));
        }
    }
    sort(v.begin(), v.end(), compare_info); //오름차순
 
    double *p = centroids.ptr<double>(v[0].get_index()); //현재 좌표에서 가장 가까운 객체의 무게중심 좌표
    present_point = cv::Point2d(p[0], p[1]); //현재좌표 초기화
 
    //distance = cv::sqrt(cv::pow((present_point.x - past_point.x), 2) + cv::pow((present_point.y - past_point.y), 2));
    if((cv::abs(present_point.x - past_point.x) > toUse.cols/4) 
        || (cv::abs(present_point.y - past_point.y) > toUse.rows/2)) {
        present_point = past_point;
    }
 
    error.data = static_cast<int>(toUse.cols/2 - present_point.x);
    RCLCPP_INFO(this->get_logger(), "error : %d", error);
 
    //Blue
    for(int j = 1; j < (int)v.size(); j++) {
        double *p = centroids.ptr<double>(v[j].get_index());
        int *q = stats.ptr<int>(v[j].get_index());
        circle(toUse, cv::Point(p[0],p[1]), 3, cv::Scalar(255,0,0), -1);
        rectangle(toUse, cv::Rect(q[0], q[1], q[2], q[3]), cv::Scalar(255,0,0));
    }
    //Red Point
    circle(toUse, present_point, 3, cv::Scalar(0,0,255), -1);
 
    cv::imshow("original", original);
    cv::imshow("toUse", toUse);
    cv::waitKey(1);
 
    original_video << original;
    toUse_video << toUse;
 
    past_point = present_point; //과거좌표 초기화
}
 
void wsl::pub_callback() //error pub
{
    pub->publish(error);
}
 
wsl::wsl() : Node("wsl"),
    original_video("original.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 360)),
    toUse_video("toUse.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 90)),
    past_point(320, 45), present_point(320, 45)
{
    sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("camera", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&wsl::img_show, this, _1));
 
    pub = this->create_publisher<std_msgs::msg::Int32>("error",
        rclcpp::QoS(rclcpp::KeepLast(10)));
    timer_ = this->create_wall_timer(30ms, std::bind(&wsl::pub_callback, this));
}