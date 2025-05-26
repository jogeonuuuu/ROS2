#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
// #include "dxl_wsl/dxl.hpp"
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

/*bool kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt); // 현재 터미널 속성 저장
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 버퍼 없이 입력, 입력 문자 출력 안함
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // 논블로킹 모드

    ch = getchar(); // 키 입력 읽기
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 원래 터미널 속성 복원
    fcntl(STDIN_FILENO, F_SETFL, oldf); // 원래 파일 속성 복원

    if (ch != EOF) {
        ungetc(ch, stdin); // 읽은 키를 다시 입력 스트림으로 돌려놓음
        return true;
    }

    return false;
}

char getch() {
    struct termios oldt, newt;
    char ch;

    tcgetattr(STDIN_FILENO, &oldt); // 현재 터미널 속성 저장
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 버퍼 없이 입력, 입력 문자 출력 안함
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar(); // 실제 키 입력
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 원래 터미널 속성 복원

    return ch;
}*/

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_dxlpub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
    geometry_msgs::msg::Vector3 vel;
    vel.x = vel.y = vel.z = 0;
    
    rclcpp::WallRate loop_rate(20.0); //20hz->50msec
    int vel1=0, vel2=0;
    int goal1=0, goal2=0;
    while(rclcpp::ok())
    {
        /*if (kbhit())
        {
            char c = getch();
            switch(c)
            {
            case 's': goal1 = 0; goal2 = 0; break;
            case ' ': goal1 = 0; goal2 = 0; break;
            case 'f': goal1 = 50; goal2 = -50; break;
            case 'b': goal1 = -50; goal2 = 50; break;
            case 'l': goal1 = -50; goal2 = -50; break;
            case 'r': goal1 = 50; goal2 = 50; break;
            default : goal1 = 0; goal2 = 0; break;
            }
        }
        // generate accel and decel motion
        if(goal1>vel1) vel1 += 5;
        else if(goal1<vel1) vel1 -= 5;
        else vel1 = goal1;
        if(goal2>vel2) vel2 += 5;
        else if(goal2<vel2) vel2 -= 5;
        else vel2 = goal2;
        vel.x = vel1;
        vel.y = vel2;*/
        /*
        std::cout << "input >> ";
        std::string str;
        std::cin >> str;
        if(!str.empty())
        {
            c=str[0];
            switch(c)
            {
            case ' ': vel.x = 0; vel.y = 0; break; //정지
            case 'w': vel.x = 50; vel.y = -50; break; //전진
            case 's': vel.x = -50; vel.y = 50; break; //후진
            case 'a': vel.x = -50; vel.y = -50; break; //좌회전
            case 'd': vel.x = 50; vel.y = 50; break; //우회전
            default : vel.x = 0; vel.y = 0; break;
            }
        }*/
        std::cout << "input >> ";
        char c='p';
        std::cin >> c;
        if(c!='p')
        {switch(c)
        {
        case ' ': vel.x = 0; vel.y = 0; break; //정지
        case 'w': vel.x = 50; vel.y = -50; break; //전진
        case 's': vel.x = -50; vel.y = 50; break; //후진
        case 'a': vel.x = -50; vel.y = -50; break; //좌회전
        case 'd': vel.x = 50; vel.y = 50; break; //우회전
        default : vel.x = 0; vel.y = 0; break;
        }}

        RCLCPP_INFO(node->get_logger(), "Publish: %lf,%lf", vel.x, vel.y);
        mypub->publish(vel);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}