
# 빌드 방법

### 윈도우에서서 우분투 위치 찾기
```
\\wsl$\Ubuntu-20.04\
```

### 빌드
```sh
cd ~/xycar_ws
catkin_make
source devel/setup.bash
```

### TCP 통신 수행
```sh
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10000
```
### 실행
```sh
python3 ~/xycar_ws/src/kookmin/driver/track_drive.py
```