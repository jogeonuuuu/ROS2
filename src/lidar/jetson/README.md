[slamtec lidar ros2](https://github.com/Slamtec/sllidar_ros2)    
[code used](https://github.com/Slamtec/sllidar_ros2/blob/main/src/sllidar_node.cpp)

- **Lidar Model:** RPLIDAR [C1](https://github.com/Slamtec/sllidar_ros2/blob/main/launch/sllidar_c1_launch.py)
```
// sllidar_node.cpp (line 69~80)
this->declare_parameter<std::string>("channel_type","serial");
this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");
this->declare_parameter<int>("tcp_port", 20108);
this->declare_parameter<std::string>("udp_ip","192.168.11.2");
this->declare_parameter<int>("udp_port",8089);
this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0"); -> 장치명 작성
this->declare_parameter<int>("serial_baudrate",1000000); -> ('serial_baudrate', default='460800')
this->declare_parameter<std::string>("frame_id","laser_frame"); -> ('frame_id', default='laser')
this->declare_parameter<bool>("inverted", false);
this->declare_parameter<bool>("angle_compensate", false); -> ('angle_compensate', default='true')
this->declare_parameter<std::string>("scan_mode",std::string()); -> ('scan_mode', default='Standard')
this->declare_parameter<float>("scan_frequency",10);
```
- **Node:** sllidar_node   
- **Interface:** sensor_msgs/msg/LaserScan
  - ``$ ros2 interface show sensor_msgs/msg/LaserScan`` scan_time # time between scans [seconds]    
- **Topic:** /scan
  - ``$ ros2 topic hz /scan`` 평균 속도: 10Hz (초당 100ms)
  - ``$ ros2 topic bw /scan`` 크기: 58.88 KB/s (장당 5.88 KB)
  - ``$ ros2 topic echo /scan`` 내용 중.. ``scan_time: 0.100168..`` -> lidar가 한바퀴 도는데 걸리는 시간이 0.1sec (10Hz)라는 것을 확인할 수 있음.
