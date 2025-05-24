### Publisher Node
**① rclcpp::WallRate**
```
main()
  rclcpp::WallRate loop_rate(1.0) //반복주파수를 저장하는 객체(1.0Hz)

while()
  loop_rate.sleep(): //반복주파수에서 남은 시간만큼 sleep
```

**② rclcpp::sleep_for**
```
#include <chrono>
using namespace std::chrono_literals;

while()
  rclcpp::sleep_for(chrono::nanoseconds(1s)); //rclcpp::WallRate(loop_rate, loop_rate.sleep())을 하나로
```
