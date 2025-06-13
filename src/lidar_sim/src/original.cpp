#include "lidar_sim/simulation.hpp"
 
cv::Mat Simulation::lidar_img(500, 500, CV_8UC3, cv::Scalar(255,255,255));
cv::VideoWriter Simulation::output("lidar_sim_pathway.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 6, cv::Size(500, 500));
//fps: 6 ($ ros2 topic hz <토픽명> (/scan))
std_msgs::msg::Int32 Simulation::error;
//double Simulation::left_min_distance = 100;
//double Simulation::right_min_distance = 100;

void Simulation::scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int count = scan->scan_time / scan->time_increment;
    //printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    //printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),RAD2DEG(scan->angle_max));
    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); //RAD2DEG(x) ((x)*180./M_PI)
        float distance = scan->ranges[i];
        //printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, distance);
        
        // 스캔영상 그리기 (500pxl x 500pxl / 10m x 10m) (-> 1m 당 50pxl)
        float rad = DEG2RAD(degree); //DEG2RAD(x) (x * M_PI/180.)
        float meter = 4; float radius = MET2PXL(meter); // 반경
        int xpixel = 250 + (sin(rad) * distance * radius);
        int ypixel = 250 + (cos(rad) * distance * radius);
        cv::rectangle(lidar_img, cv::Rect(xpixel, ypixel, 3, 3), cv::Scalar(0,0,255), -1);
    }
    cv::drawMarker(lidar_img, cv::Point(lidar_img.cols/2 - 1, lidar_img.rows/2 - 1), cv::Scalar(25,0,0));
    error_calc(lidar_img); //에러값 계산 함수
 
    cv::imshow("lidar", lidar_img);
    cv::waitKey(1);
    output << lidar_img; //영상 저장
 
    //화면 초기화
    lidar_img = cv::Scalar(255,255,255);
}

void Simulation::error_calc(cv::Mat &lidar_img) { //error_value calculator
    cv::Mat toUse = lidar_img(cv::Rect(0, 0, lidar_img.cols, lidar_img.rows/2));
    cv::cvtColor(toUse, toUse, cv::COLOR_BGR2GRAY);
    cv::threshold(toUse, toUse, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU); // 이진화
    
    //왼쪽 상단 영역 정보(제 2사분면)
    cv::Mat left_area = toUse(cv::Rect(0, 0, toUse.cols/2 -1, toUse.rows- 1)); //'-1': 중앙점 제거
    cv::Point left_min_point(lidar_img.cols/2, lidar_img.rows/2);
    double left_min_distance = sqrt(pow(left_area.cols, 2) + pow(left_area.rows, 2)); //장애물과의 최단거리
    //오른쪽 상단 영역 정보(제 1사분면)
    cv::Mat right_area = toUse(cv::Rect(toUse.cols/2, 0, toUse.cols/2 -1, toUse.rows -1));
    cv::Point right_min_point(lidar_img.cols/2, lidar_img.rows/2);
    double right_min_distance = sqrt(pow(right_area.cols, 2) + pow(right_area.rows, 2)); //장애물과의 최단거리
 
    
    cv::Mat labels, stats, centroids; // labels: 객체에 번호가 지정된 레이블 맵
                                      // stats: x,y(좌측 상단 좌표), width,height, area(면적: 픽셀 수)
                                      // centroids: 무게 중심 좌표 (rows, cols)
    int lable_cnt;
    //Left Area minimum distance
    //bool left_ox = false;
    lable_cnt = cv::connectedComponentsWithStats(left_area, labels, stats, centroids);
    cv::cvtColor(left_area, left_area, cv::COLOR_GRAY2BGR);
    for(int i = 1; i < lable_cnt; i++) {
        double *p = centroids.ptr<double>(i); //centroids 행렬의 i번째 행의 주소
        //최소값 비교
        double compare = sqrt(pow((left_area.cols-1 - p[0]), 2) + pow((left_area.rows-1 - p[1]), 2));
        if(left_min_distance > compare) {
            left_min_distance = compare;
            left_min_point = cv::Point(p[0], p[1]);
        }
        //left_ox = true;
 
        //test
         //int *q = stats.ptr<int>(i);
         //cv::rectangle(left_area, cv::Rect(q[0],q[1],q[2],q[3]), cv::Scalar(0,255,255), 2);
    }
    //Right Area minimum distance
    //bool right_ox = false;
    lable_cnt = cv::connectedComponentsWithStats(right_area, labels, stats, centroids);
    cv::cvtColor(right_area, right_area, cv::COLOR_GRAY2BGR);
    for(int i = 1; i < lable_cnt; i++) {        
        double *p = centroids.ptr<double>(i);
        //최소값 비교
        double compare = sqrt(pow(p[0], 2) + pow((right_area.rows-1 - p[1]), 2));
        if(right_min_distance > compare) {
            right_min_distance = compare;
            right_min_point = cv::Point(p[0], p[1]);
        }
        //right_ox = true;
 
        //test
         //int *q = stats.ptr<int>(i);
         //cv::rectangle(right_area, cv::Rect(q[0],q[1],q[2],q[3]), cv::Scalar(0,255,255), 2);
    }
 
    //test
     //cv::line(left_area, cv::Point(left_area.cols-1, left_area.rows-1), left_min_point, cv::Scalar(0,0,255));
     //cv::line(right_area, cv::Point(0, right_area.rows-1), right_min_point, cv::Scalar(0,0,255));
     //cv::imshow("left", left_area);
     //cv::imshow("right", right_area);
 
    double left_angle = RAD2DEG(atan2(lidar_img.rows/2 - left_min_point.y, lidar_img.cols/2 - left_min_point.x)) - 90; //-90: θ값은 lidar x축 기준으로 반시계 방향이므로 => 음수
    double right_angle = 90 - RAD2DEG(atan2(lidar_img.rows/2 - right_min_point.y, right_min_point.x));
    double center_angle = (left_angle + right_angle) / 2.0;
    printf("Left Angle, Right Angle : %lf, %lf\n", left_angle, right_angle);
    printf("Center Angle: %lf\n", center_angle);
 
 
    //아래의 조건 필요 X -> 'left_min_point'와 'right_min_point' 객체를 초기화해주므로써
    //if(left_ox && right_ox) error.data = center_angle; //양쪽 영역 모두 객체가 있는 경우
    //else if(left_ox) error.data = (left_angle + 90) / 2; //왼쪽 영역만 객체가 있는 경우 (&& !right_ox)
    //else if(right_ox) error.data = (right_angle - 90) / 2; //오른쪽 영역만 객체가 있는 경우 (&& !left_ox)
    //else error.data = 0; //양쪽 영역 모두 객체가 없는 경우
 
    //left_ox = right_ox = false; //초기화
    
 
    //왼쪽 장애물(초록), 오른쪽 장애물(파랑)
    cv::line(lidar_img, cv::Point(lidar_img.cols/2, lidar_img.rows/2), left_min_point, cv::Scalar(0,255,0));
    cv::line(lidar_img, cv::Point(lidar_img.cols/2, lidar_img.rows/2), 
        cv::Point(lidar_img.cols/2 + right_min_point.x, right_min_point.y), cv::Scalar(255,0,0));
    //진행방향(화살표)
    int center_xpoint = RAD2DEG(sin(center_angle * M_PI/180)) * 5/3;
    int center_ypoint = RAD2DEG(cos(center_angle * M_PI/180)) * 5/3;
    //printf("Center Point : %d, %d\n", center_xpoint, center_ypoint);
    cv::arrowedLine(lidar_img, cv::Point(lidar_img.cols/2, lidar_img.rows/2),
        cv::Point(lidar_img.cols/2 + center_xpoint, lidar_img.rows/2 - center_ypoint), cv::Scalar(0,0,0));
}
 
void Simulation::pub_callback() {
    pub->publish(error);
    //RCLCPP_INFO(this->get_logger(), "error: %d", error.data);
}

Simulation::Simulation() : Node("sllidar_client")
{
    lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",
        rclcpp::SensorDataQoS(), scanCb); //std::bind(&Simulation::scanCb, this, _1) -> scanCb

    pub = this->create_publisher<std_msgs::msg::Int32>("error", rclcpp::QoS(rclcpp::KeepLast(10)));
    timer_ = this->create_wall_timer(10ms, std::bind(&Simulation::pub_callback, this));
}