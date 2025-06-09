#include "psub2-1/pub.hpp"
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pub>();
    rclcpp::spin(node); //이벤트를 체크하고 발생시 콜백함수를 호출
                        //이벤트가 발생하지 않으면 무한히 대기
 
    rclcpp::shutdown();
    return 0;
}