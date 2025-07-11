# ROS2

### wsl2 20.04 install   
~~$ wsl --list --online ($ wsl -l -o)~~   
$ wsl --install -d Ubuntu-20.04

</br></br>

### [foxy install](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)   
```
# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update && sudo apt upgrade

sudo apt install ros-foxy-desktop python3-argcomplete

sudo apt install ros-dev-tools
```
설치 후 아래 명령어 실행 후 도움말이 나오는지 확인.
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
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
 
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/robot_ws
 
export ROS_DOMAIN_ID=7
export ROS_NAMESPACE=robot1
 
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
 
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1
 
alias cbp='colcon build --symlink-install --packages-select'
```
1. ROS 2 및 기타 패키지 환경 설정 소스 명령어
    - line 1: ROS 2 Humble 환경 설정.
    - line 2: 사용자의 로봇 작업 공간을 설정. (디렉토리명에 따라 수정)
    - line 4~6:  colcon 빌드 시스템 및 버전 관리 도구에 대한 자동 완성 및 명령어 도우미 기능 활성화.

2. 환경 변수 설정
    - line 7: colcon 작업 공간의 루트 디렉터리를 설정.
    - line 9: jetson보드의 ROS 네트워크의 도메인 ID 설정. (ROS 시스템 간의 통신 격리에 사용) ($ echo $ROS_DOMAIN_ID)
    - line 10:  기본 ROS 네임스페이스 설정.
    - line 12:  ROS 미들웨어 구현을 선택.

3. 로깅 포맷 및 설정
    - line 17~20: 로그 메시지의 형식 설정  /  로그 출력 색상 사용 여부 설정  /  표준 출력 대신 로그 파일을 사용할지 여부 설정  /  로그 메시지를 버퍼링할지 여부 설정

4. 별명(alias) 설정: line 22

</br></br>


