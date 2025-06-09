> **pub1-1**
> - 정수값을 0으로 초기화하고 1씩 증가시키면서 퍼블리시하는 패키지
> - Interface(topic massege): std_msgs/msg/Int32 (헤더파일명, 클래스명)

<br/>

> **pub1-2**
> - 키보드로부터 실수값 3개를 입력 받아 퍼블리시하는 패키지
> - Interface(topic massege): geometry_msgs/msg/Vector3 (헤더파일명, 클래스명)

<br/>

> **pub1-3**
> - Turtlesim 패키지의 teleop_turtle 노드를 대신하는 패키지
>   - Turtlesim 패키지의 turtle-teleop_key 실행파일에 지정된 노드
> - 키보드 입력시 /turtle1/cmd_vel 토픽 발행
>   - Subscriber 역할을 수행하는 Turtlesim 패키지의 turtle-teleop_key 실행파일을 수행했을 때 발행되는 토픽
> - Interface(topic massege): geometry_msgs/msg/Twist (헤더파일명, 클래스명)
