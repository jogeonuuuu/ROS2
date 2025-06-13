#include "lidar_sim/simulation.hpp"

cv::Mat Simulation::lidar_img(500, 500, CV_8UC3, cv::Scalar(255,255,255));
cv::VideoWriter Simulation::output("lidar_sim_pathway.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 6, cv::Size(500, 500)); //fps: 6 ($ ros2 topic hz <토픽명> (/scan))
std_msgs::msg::Int32 Simulation::error;

void Simulation::error_calc(cv::Mat &lidar_img) { //error_value calculator
    cv::Mat toUse = lidar_img(cv::Rect(0, 0, lidar_img.cols, lidar_img.rows/2));
    cv::cvtColor(toUse, toUse, cv::COLOR_BGR2GRAY);
    cv::threshold(toUse, toUse, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU); // 이진화

    //왼쪽 상단 영역 정보(제 2사분면)
    cv::Mat left_area = toUse(cv::Rect(0, 0, toUse.cols/2 -1, toUse.rows- 1)); //'-1': 중앙점 제거
    cv::Point left_min_point(lidar_img.cols/2, lidar_img.rows/2);
    double left_min_distance = sqrt(pow(left_area.cols, 2) + pow(left_area.rows, 2)); //최단거리 장애물과의 거리

    //오른쪽 상단 영역 정보(제 1사분면)
    cv::Mat right_area = toUse(cv::Rect(toUse.cols/2, 0, toUse.cols/2 -1, toUse.rows -1));
    cv::Point right_min_point(lidar_img.cols/2, lidar_img.rows/2);
    double right_min_distance = sqrt(pow(right_area.cols, 2) + pow(right_area.rows, 2)); //최단거리 장애물과의 거리

    cv::Mat labels, stats, centroids; // labels: 객체에 번호가 지정된 레이블 맵
                                      // stats: x,y(좌측 상단 좌표), width,height, area(면적: 픽셀 수)
                                      // centroids: 무게 중심 좌표 (rows, cols)
    int lable_cnt;

    //Left Area minimum distance
    //bool left_ox = false;
    lable_cnt = cv::connectedComponentsWithStats(left_area, labels, stats, centroids);
    cv::cvtColor(left_area, left_area, cv::COLOR_GRAY2BGR);
    for(int i = 1; i < lable_cnt; i++) {
        int *p = stats.ptr<int>(i);
        double compare = sqrt(pow((left_area.cols-1 - (p[0]+p[2])), 2) + pow((left_area.rows-1 - (p[1]+p[3])), 2));
        if(left_min_distance > compare) {
            left_min_distance = compare;
            left_min_point = cv::Point((p[0]+p[2]), (p[1]+p[3]));
        }

        //test
        //int *q = stats.ptr<int>(i);
        cv::rectangle(left_area, cv::Rect(p[0],p[1],p[2],p[3]), cv::Scalar(0,255,255), 2);
    }

    //Right Area minimum distance
    //bool right_ox = false;
    lable_cnt = cv::connectedComponentsWithStats(right_area, labels, stats, centroids);
    cv::cvtColor(right_area, right_area, cv::COLOR_GRAY2BGR);
    for(int i = 1; i < lable_cnt; i++) {
        int *p = stats.ptr<int>(i);
        double compare = sqrt(pow((p[0]), 2) + pow((right_area.rows-1 - (p[1]+p[3])), 2));
        if(right_min_distance > compare) {
            right_min_distance = compare;
            right_min_point = cv::Point(p[0], (p[1]+p[3]));
        }

        //test
        //int *q = stats.ptr<int>(i);
        cv::rectangle(right_area, cv::Rect(p[0],p[1],p[2],p[3]), cv::Scalar(0,255,255), 2);
    }

    //test
    cv::line(left_area, cv::Point(left_area.cols-1, left_area.rows-1), left_min_point, cv::Scalar(0,0,255));
    cv::line(right_area, cv::Point(0, right_area.rows-1), right_min_point, cv::Scalar(0,0,255));
    cv::imshow("left", left_area);
    cv::imshow("right", right_area);

    double left_angle = RAD2DEG(atan2(lidar_img.rows/2 - left_min_point.y, lidar_img.cols/2 - left_min_point.x)) - 90;
    double right_angle = 90 - RAD2DEG(atan2(lidar_img.rows/2 - right_min_point.y, right_min_point.x)); // x축: (right_min_point.x - lidar_img.cols/2)
    double center_angle = (left_angle + right_angle) / 2.0;
    printf("Left Angle, Right Angle : %lf, %lf\n", left_angle, right_angle);
    printf("Center Angle: %lf\n", center_angle);
    //에러값 저장
    error.data = -(static_cast<int>(center_angle));

    //아래의 조건 필요 X -> 'left_min_point'와 'right_min_point' 객체를 초기화해주므로써
    //if(left_ox && right_ox) error.data = center_angle;
    //else if(left_ox) error.data = (left_angle + 90) / 2;
    //else if(right_ox) error.data = (right_angle - 90) / 2;
    //else error.data = 0;
    //left_ox = right_ox = false;


    //왼쪽 장애물(초록)
    cv::line(lidar_img, cv::Point(lidar_img.cols/2, lidar_img.rows/2), left_min_point, cv::Scalar(0,255,0));
    cv::circle(lidar_img, left_min_point, 3, cv::Scalar(0,255,0), -1);
    //오른쪽 장애물(파랑)
    cv::line(lidar_img, cv::Point(lidar_img.cols/2, lidar_img.rows/2), 
        cv::Point(lidar_img.cols/2 + right_min_point.x, right_min_point.y), cv::Scalar(255,0,0));
     cv::circle(lidar_img,
         cv::Point((lidar_img.cols/2 -1) + right_min_point.x, right_min_point.y), 3, cv::Scalar(255,0,0), -1);

    //진행방향(화살표)
    int center_xpoint = RAD2DEG(sin(center_angle * M_PI/180)) * 5/3;
    int center_ypoint = RAD2DEG(cos(center_angle * M_PI/180)) * 5/3;
    //printf("Center Point : %d, %d\n", center_xpoint, center_ypoint);
    cv::arrowedLine(lidar_img, cv::Point(lidar_img.cols/2, lidar_img.rows/2),
        cv::Point(lidar_img.cols/2 + center_xpoint, lidar_img.rows/2 - center_ypoint), cv::Scalar(0,0,0));
}

// void Simulation::pub_callback() {
//     // pub->publish(error);
//     //RCLCPP_INFO(this->get_logger(), "error: %d", error.data);
// }

Simulation::Simulation() : Node("sllidar_client")
{
    cv::VideoCapture cap("/home/linux/ros2_ws/lidar_save.mp4");
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "video open error");
        return;
    }

    cv::Mat frame;
    while (rclcpp::ok() && cap.read(frame)) {
        lidar_img = frame.clone(); // 프레임을 전역 이미지로 설정
        error_calc(lidar_img); // 에러 계산 함수

        cv::imshow("Lidar View", lidar_img);
        cv::waitKey(1); // OpenCV용 GUI 이벤트 처리용

        output << lidar_img; // 영상 저장

        pub = this->create_publisher<std_msgs::msg::Int32>("error", rclcpp::QoS(rclcpp::KeepLast(10)));
        pub->publish(error);
        RCLCPP_INFO(this->get_logger(), "error: %d", error.data);

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz 맞춤
    }
}