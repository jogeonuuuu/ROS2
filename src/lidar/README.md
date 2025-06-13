## Question. 

**1. Rplidar C1의 좌표축을 설명하라.**
  [참고 파일](https://bucket-download.slamtec.com/c5971f2703a8d014f3925694d798ea490a370efa/LR001_SLAMTEC_rplidar_S&C%20series_protocol_v2.8_en.pdf)   
  <img src="https://github.com/user-attachments/assets/b1623359-9d04-4040-9949-fb1c2f0fefdf" width="50%" height="50%">   
  라이다 기준 정면이 x축

**2. Rplidar C1은 1초에 몇 번 토픽메시지를 전송하는가?**
10hz

**3. 토픽메시지의 크기(KB)는 얼마인가?**
![image](https://github.com/user-attachments/assets/6a3a1695-323a-43bb-ae5b-942329ed9f1a)   
한장당 5.82KB


**4. 메시지 1개당 몇 개의 거리 측정값이 포함되어 있는가?** && **5. 1회전에 몇 번 거리를 측정하는가?**    
![image](https://github.com/user-attachments/assets/4db10f2b-efbf-40a3-a9c9-fc678b0ed5a1)    
0.00873/0.0001387 값


**6. angle_min, angle_increment 값은 얼마인가?**   
angle_min: 라이다가 한 바퀴 도는 값   
angle_increment: 라이다가 한 좌표를 찍고 다음 좌표를 찍는 사이의 값
