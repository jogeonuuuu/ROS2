# ROS2

### wsl2 20.04 install   
~~$ wsl --list --online ($ wsl -l -o)~~   
wsl --install -d Ubuntu-20.04

</br></br>
[foxy install](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
후 아래 명령어 실행 후 도움말이 나오는지 확인.
```
$ source /opt/ros/humble/setup.bash
$ ros2

$ source /opt/ros/humble/setup.bash
$ colcon --help
```

</br></br>

### ~/.bashrc 파일 편집
[참고 사이트](https://cafe.naver.com/openrt/25288)

```
source /opt/ros/foxy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/
colcon build --symlink-install
```

```
source /opt/ros/humble/setup.bash # ROS2 Humble 환경 설정
source ~/ros2_ws/install/local_setup.bash # 사용자의 로봇 작업 공간을 설정. (디렉토리명에 따라 수정)

# colcon 빌드 시스템 및 버전 관리 도구에 대한 자동 완성 및 명령어 도우미 기능 활성화
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh

# 환경변수 설정
export _colcon_cd_root=~/ros2_ws # colcon 작업 공간의 루트 디렉터리를 설정
export ROS_DOMAIN_ID=7 # jetson보드의 ROS 네트워크의 도메인 ID 설정. (ROS 시스템 간의 통신 격리에 사용) ($ echo $ROS_DOMAIN_ID)
export ROS_NAMESPACE=robot1 # 기본 ROS 네임스페이스 설정

# ROS 미들웨어 구현을 선택
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

# 로그 메시지의 형식 설정 / 로그 출력 색상 사용 여부 설정 / 표준 출력 대신 로그 파일을 사용할지 여부 설정 / 로그 메시지를 버퍼링할지 여부 설정
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1
 
alias cbp='colcon build --symlink-install --packages-select'
```
