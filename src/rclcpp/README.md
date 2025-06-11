## 개발 환경: WSL2: Ubuntu20.04 (with ROS2 Foxy ver.)

<br/>

|**<Node 객체 생성 및 Publiser 객체 생성>**|**<추가로 Subscription 객체 생성 후 통신 테스트>**|**<분할 컴파일>**|
|:---:|:---:|:---:|
|[rclcpp-1](https://github.com/jogeonuuuu/ROS2/tree/main/src/rclcpp/rclcpp-1): ``rclcpp::WallRate`` ``rclcpp::sleep_for()`` </br>[rclcpp-2](https://github.com/jogeonuuuu/ROS2/tree/main/src/rclcpp/rclcpp-2): ``rclcpp::Node::create_wall_timer(), spin()``|[rclcpp-3](https://github.com/jogeonuuuu/ROS2/tree/main/src/rclcpp/rclcpp-3)|[rclcpp-4](https://github.com/jogeonuuuu/ROS2/tree/main/src/rclcpp/rclcpp-4)

<br/><br/>

### **rclcpp-1**
**① rclcpp::WallRate** ``loop_rate(), loop_rate.sleep()``
   ```cpp
   rclcpp::WallRate loop_rate(1.0); //반복 주파수를 저장하는 객체(1.0 Hz).
   ''' (생략)
   while(rclcpp::ok()) {
     ''' (생략)
     loop_rate.sleep(); //반복 주파수에서 남은 시간 만큼 대기.
   }
   ```
**② rclcpp::sleep_for** ``rclcpp::sleep_for()``
   ```cpp
   #include <chrono>
   using namespace std::chrono_literals; //숫자와 단위를 포함한 사용자 정의 리터럴을 사용할 때. 
   ''' (생략)
   while(rclcpp::ok()) {
     ''' (생략)
     rclcpp::sleep_for(chrono::nanoseconds(1s)); //rclcpp::WallRate(loop_rate(), loop_rate.sleep())을 하나로 묶은 개념.
   }
   ```

<br/>

### **rclcpp-2** ``rclcpp::Node::create_wall_timer(), spin()``
<img src="https://github.com/user-attachments/assets/d3780af5-8842-4cb9-8aa5-d3459860ef8a" width="50%" height="50%">
<img src="https://github.com/user-attachments/assets/8a4948e7-f954-43bc-ae01-741eba9d558e" width="50%" height="50%">

   ```
   rclcpp::WallTimer 클래스: 토픽 통신시 주기적으로 메시지를 전송하고 싶을 때 주기적인 타이머 이벤트를 발생시키는 클래스.
   rclcpp::Node::create_wall_timer 함수를 이용하여 동적객체를 생성.
   ```
   ```cpp
   #include <chrono>
   #include <functional> //std::function<void()>
   using namespace std::chrono_literals;
   ''' (생략: callback함수 정의)
   std::function<void()> fn = std::bind(callback, node, pub); //callback함수 들어갈 인자들(node(node_name), pub(topic_name)).
   rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(50ms, std::bind()); //두 번째 매개변수인 CallBackT의 자료형은 std::function<void()>로 만들어야 됨.
                                                                                     //타이머 이벤트 주기와 콜백함수가 등록된 shared_ptr(SharedPtr) 객체
                                                                                     //TimerBase인 이유: ?
                                                                                     //std::shared_ptr<X> = X:SharedPtr
   rclcpp::spin(node); //"node"가 가리키는 노드의 실행을 무한히 반복.
   ```

<br/>

### **rclcpp-3**
<img src="https://github.com/user-attachments/assets/92d33fcf-35e2-4313-b90e-0a82260a8b45" width="50%" height="50%">


<br/><br/>
-----------------------------------------------------------------------
<br/><br/>


[ROS2 foxy Document](https://docs.ros.org/en/foxy/index.html) / [rclcpp](https://docs.ros2.org/foxy/api/rclcpp/index.html)

**_Message Interface_**   
소스코드에서 message interface headerfile은 별도 포함해야 됨. (ex: `#include "std_msgs/msg/string.hpp“`)   
**설치 경로:** /opt/ros/foxy/(std_msgs, geometry_msgs, sensor_msgs)/msg/*.hpp

<img src="https://github.com/user-attachments/assets/3ff140fd-7569-405d-9e26-9ecc628105bd" width="70%" height="70%">
