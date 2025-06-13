## [개발 환경] <br/> ``Jetson nano board``: Ubuntu20.04 (with ROS2 Foxy ver.)

### 수신한 영상을 컬러영상 -> 그레이영상 -> 이진영상으로 변환하고 이진영상을 PC로 전송하여 출력하는 패키지

<br/>

- **Publisher Node**
  - **Node name:** campub
  - **Topic name:** image/compressed
  - **Interface(topic message):** sensor_msgs::msg::CompressedImage
    - ``cv_bridge::CvImage::toCompressedImageMsg()``   
      -> cv::Mat 객체를 sensor_msgs::msg::CompressedImage 객체로 변환시켜주는 함수 (기본 압축 포맷: jpg)
  - **프로토콜:** UDP (영상 통신시 속도가 중요하므로 Qos설정에서 best effort 모드(UDP) 사용)
  - **반복 주기**
    - 40Hz (csi camera 성능상 최대 30Hz 지원)   
    - ``$ ros2 topic hz /image/compressed`` "average rate: 30.016"
- **Subscriber Node**
  - **Node name:** camsub
  - **Topic name:** image/compressed
  - **Interface(topic message):** sensor_msgs::msg::CompressedImage
    - ``cv_bridge::toCvShare()``   
      -> sensor_msgs::msg::CompressedImage 객체를 cv::Mat 객체로 변환시켜주는 함수
  - **프로토콜:** UDP (영상 통신시 속도가 중요하므로 Qos설정에서 best effort 모드(UDP) 사용)
  - **반복 주기**
    - ``spin()``함수를 통해``node``가 가리키는 <ins>노드(camsub)의 실행</ins>을 무한히 반복.
    - **노드(camsub)의 실행:** Publisher Node에서 전송하는 Topic인 ``image/compressed``를 수신했을 때 호출되는 콜백함수(``mysub_callback``)
   
