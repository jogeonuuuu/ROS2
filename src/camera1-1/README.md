## 개발 환경: Jetson nano board: Ubuntu20.04 (with ROS2 Foxy ver.)

수신한 영상을 컬러영상 -> 그레이영상 -> 이진영상으로 변환하고 이진영상을 PC로 전송하여 출력하는 패키지

- **Publisher Node**
  - **node name:** campub
  - **topic name:** image/compressed
  - **프로토콜:** UDP (영상 통신시 속도가 중요하므로 Qos설정에서 best effort 모드(UDP) 사용)
  - **반복 주기**
    - 40Hz (csi camera 성능상 최대 30Hz 지원)   
    - ``$ ros2 topic hz /image/compressed`` average rate: 30.016
- **Subscriber Node**
  - **node name:** camsub
  - **topic name:** image/compressed
  - **프로토콜:** UDP (영상 통신시 속도가 중요하므로 Qos설정에서 best effort 모드(UDP) 사용)
  - **반복 주기**
    - ``spin()``함수를 통해``node``가 가리키는 <ins>노드(camsub)의 실행</ins>을 무한히 반복.
    - **노드(camsub)의 실행:** Publisher Node에서 전송하는 Topic인 ``image/compressed``를 수신했을 때 호출되는 콜백함수(``mysub_callback``)
   
